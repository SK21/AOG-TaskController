/**
 * @author Daan Steenbergen
 * @brief An ISOBUS Task Controller for AgOpenGPS
 * @version 0.1
 * @date 2025-1-20
 *
 * @copyright 2025 Daan Steenbergen
 */
#include "task_controller.hpp"
#include "logging_utils.hpp"
#include "settings.hpp"

#include "isobus/isobus/isobus_device_descriptor_object_pool_helpers.hpp"
#include "isobus/isobus/isobus_task_controller_server.hpp"
#include "isobus/utility/system_timing.hpp"

#include <algorithm>
#include <bitset>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

/// Delay between pool activation and applying the initial section control state, so the implement
/// has settled and answered the measurement commands before it is switched to automatic.
static constexpr std::uint32_t INITIAL_SECTION_CONTROL_DELAY_MS = 2000;

// Sanitize a string for use as a filename by replacing invalid characters with underscores
static std::string sanitize_filename(const std::string &input)
{
	std::string result = input;
	// Characters invalid on Windows (and / is invalid on POSIX too)
	const std::string invalid_chars = "\\/*:?\"<>|";
	for (char &c : result)
	{
		if (invalid_chars.find(c) != std::string::npos)
		{
			c = '_';
		}
	}
	return result;
}

// Finds the number of the first device element that lists the process data object as a child.
static bool find_owning_element_number(isobus::DeviceDescriptorObjectPool &pool,
                                       const isobus::task_controller_object::DeviceProcessDataObject &processData,
                                       std::uint16_t &elementNumber)
{
	for (std::uint32_t i = 0; i < pool.size(); i++)
	{
		auto object = pool.get_object_by_index(i);
		if (!object || object->get_object_type() != isobus::task_controller_object::ObjectTypes::DeviceElement)
		{
			continue;
		}

		auto elementObject = std::dynamic_pointer_cast<isobus::task_controller_object::DeviceElementObject>(object);
		if (!elementObject)
		{
			continue;
		}

		for (std::uint16_t childId : elementObject->get_child_object_ids())
		{
			if (childId == processData.get_object_id())
			{
				elementNumber = elementObject->get_element_number();
				return true;
			}
		}
	}
	return false;
}

// The designator of the device element with the given number, "" when there is none.
static std::string element_designator(isobus::DeviceDescriptorObjectPool &pool, std::uint16_t elementNumber)
{
	std::string result;
	for (std::uint32_t i = 0; i < pool.size() && result.empty(); i++)
	{
		auto object = pool.get_object_by_index(i);
		if (object && object->get_object_type() == isobus::task_controller_object::ObjectTypes::DeviceElement)
		{
			auto elementObject = std::dynamic_pointer_cast<isobus::task_controller_object::DeviceElementObject>(object);
			if (elementObject && elementObject->get_element_number() == elementNumber)
			{
				result = elementObject->get_designator();
			}
		}
	}
	return result;
}

// The device element with the given element number, nullptr when there is none.
static std::shared_ptr<isobus::task_controller_object::DeviceElementObject> element_by_number(isobus::DeviceDescriptorObjectPool &pool, std::uint16_t elementNumber)
{
	std::shared_ptr<isobus::task_controller_object::DeviceElementObject> result;
	for (std::uint32_t i = 0; i < pool.size() && !result; i++)
	{
		auto object = pool.get_object_by_index(i);
		if (object && object->get_object_type() == isobus::task_controller_object::ObjectTypes::DeviceElement)
		{
			auto elementObject = std::dynamic_pointer_cast<isobus::task_controller_object::DeviceElementObject>(object);
			if (elementObject && elementObject->get_element_number() == elementNumber)
			{
				result = elementObject;
			}
		}
	}
	return result;
}

// The process data object with the given DDI among the element's children, nullptr when there is none.
static std::shared_ptr<isobus::task_controller_object::DeviceProcessDataObject> child_process_data(isobus::DeviceDescriptorObjectPool &pool,
                                                                                                     const isobus::task_controller_object::DeviceElementObject &element,
                                                                                                     std::uint16_t ddi)
{
	std::shared_ptr<isobus::task_controller_object::DeviceProcessDataObject> result;
	for (std::uint16_t childId : element.get_child_object_ids())
	{
		auto object = pool.get_object_by_id(childId);
		if (!result && object && object->get_object_type() == isobus::task_controller_object::ObjectTypes::DeviceProcessData)
		{
			auto processData = std::dynamic_pointer_cast<isobus::task_controller_object::DeviceProcessDataObject>(object);
			if (processData && processData->get_ddi() == ddi)
			{
				result = processData;
			}
		}
	}
	return result;
}

// Logs and keeps the DDOP bytes exactly as received, so pools from different sessions can be compared byte for byte.
// Files are named by hash, so each distinct pool is written once. Never affects activation.
static void save_received_ddop(std::uint64_t clientName, const std::vector<std::uint8_t> &rawPool, const std::vector<std::size_t> &chunkSizes)
{
	// FNV-1a 64-bit
	std::uint64_t hash = 0xcbf29ce484222325ULL;
	for (auto byte : rawPool)
	{
		hash = (hash ^ byte) * 0x100000001b3ULL;
	}
	std::ostringstream hashText;
	hashText << std::hex << std::setw(16) << std::setfill('0') << hash;

	std::ostringstream chunks;
	for (std::size_t i = 0; i < chunkSizes.size(); i++)
	{
		chunks << (i ? "/" : "") << chunkSizes[i];
	}

	auto fileName = std::to_string(clientName) + "/received/" + hashText.str() + ".ddop";
	auto &out = log("TC Server") << "Client " << clientName << " received DDOP: " << rawPool.size() << " bytes in "
	                             << chunkSizes.size() << " chunk(s) [" << chunks.str() << "], fnv1a64=" << hashText.str();
	try
	{
		auto path = Settings::get_filename_path(fileName);
		if (std::filesystem::exists(path))
		{
			out << " (already saved)" << std::endl;
			return;
		}
		std::ofstream outFile(path, std::ios::binary);
		outFile.write(reinterpret_cast<const char *>(rawPool.data()), static_cast<std::streamsize>(rawPool.size()));
		out << (outFile ? ", saved to " + fileName : ", failed to save " + fileName) << std::endl;
	}
	catch (const std::exception &e)
	{
		out << ", failed to save: " << e.what() << std::endl;
	}
}

void ClientState::set_number_of_sections(std::uint8_t number)
{
	numberOfSections = number;
	sectionSetpointStates.resize(number);
	sectionActualStates.resize(number);
	sectionToElementNumber.resize(number, 0); // Initialize all sections mapped to element 0 by default
}

void ClientState::set_section_setpoint_state(std::uint8_t section, std::uint8_t state)
{
	if (section < numberOfSections)
	{
		sectionSetpointStates[section] = state;
	}
}

void ClientState::set_section_actual_state(std::uint8_t section, std::uint8_t state)
{
	if (section < numberOfSections)
	{
		sectionActualStates[section] = state;
	}
}

std::uint16_t ClientState::get_element_number_for_section(std::uint8_t section) const
{
	if (section < numberOfSections && section < sectionToElementNumber.size())
	{
		return sectionToElementNumber[section];
	}
	return 0;
}

void ClientState::set_element_number_for_section(std::uint8_t section, std::uint16_t elementNumber)
{
	if (section < numberOfSections && section < sectionToElementNumber.size())
	{
		sectionToElementNumber[section] = elementNumber;
		elementToSection[elementNumber] = section;
	}
}

bool ClientState::try_get_section_for_element(std::uint16_t elementNumber, std::uint8_t &section) const
{
	auto it = elementToSection.find(elementNumber);
	if (it != elementToSection.end())
	{
		section = it->second;
		return true;
	}
	return false;
}

std::uint8_t ClientState::get_number_of_sections() const
{
	return numberOfSections;
}

std::uint8_t ClientState::get_section_setpoint_state(std::uint8_t section) const
{
	if (section < numberOfSections)
	{
		return sectionSetpointStates[section];
	}
	return SectionState::NOT_INSTALLED;
}

std::uint8_t ClientState::get_section_actual_state(std::uint8_t section) const
{
	if (section < numberOfSections)
	{
		// For legacy per-element devices, sectionActualStates is updated directly
		// from DDI 141 in on_value_command, so we can return it without the
		// expensive parent-traversal check (which has thread-safety concerns
		// when called from the heartbeat on the main thread).
		if (usesPerElementControl)
		{
			return sectionActualStates[section];
		}

		// For modern/old devices using condensed DDIs, check parent hierarchy
		std::uint16_t elementNumber = get_element_number_for_section(section);
		if (is_element_or_parent_off(elementNumber))
		{
			return SectionState::OFF;
		}
		return sectionActualStates[section];
	}
	return SectionState::NOT_INSTALLED;
}

bool ClientState::is_any_section_setpoint_on() const
{
	for (std::uint8_t state : sectionSetpointStates)
	{
		if (state == SectionState::ON)
		{
			return true;
		}
	}
	return false;
}

bool ClientState::get_setpoint_work_state() const
{
	return setpointWorkState;
}

void ClientState::set_setpoint_work_state(bool state)
{
	setpointWorkState = state;
}

bool ClientState::get_actual_work_state() const
{
	return actualWorkState;
}

void ClientState::set_actual_work_state(bool state)
{
	actualWorkState = state;
}

bool ClientState::is_section_control_enabled() const
{
	return isSectionControlEnabled;
}

void ClientState::set_section_control_enabled(bool state)
{
	isSectionControlEnabled = state;
}

void ClientState::schedule_initial_section_control(std::uint32_t dueMs)
{
	initialSectionControlDueMs = dueMs;
}

bool ClientState::take_due_initial_section_control(std::uint32_t nowMs)
{
	// Signed difference so the comparison survives timestamp wrap-around
	if (!initialSectionControlDueMs || static_cast<std::int32_t>(nowMs - *initialSectionControlDueMs) < 0)
	{
		return false;
	}
	initialSectionControlDueMs.reset();
	return true;
}

bool ClientState::uses_per_element_control() const
{
	return usesPerElementControl;
}

void ClientState::set_uses_per_element_control(bool state)
{
	usesPerElementControl = state;
}

std::uint16_t ClientState::get_per_element_setpoint_ddi() const
{
	return perElementSetpointDDI;
}

void ClientState::set_per_element_setpoint_ddi(std::uint16_t ddi)
{
	perElementSetpointDDI = ddi;
}

isobus::DeviceDescriptorObjectPool &ClientState::get_pool()
{
	return pool;
}

bool ClientState::are_measurement_commands_sent() const
{
	return areMeasurementCommandsSent;
}

void ClientState::mark_measurement_commands_sent()
{
	areMeasurementCommandsSent = true;
}

std::uint16_t ClientState::get_element_number_for_ddi(isobus::DataDescriptionIndex ddi) const
{
	auto it = ddiToElementNumber.find(ddi);
	if (it != ddiToElementNumber.end())
	{
		return it->second;
	}
	log() << "Cached element number not found for DDI " << static_cast<int>(ddi) << std::endl;
	return 0;
}

void ClientState::set_element_number_for_ddi(isobus::DataDescriptionIndex ddi, std::uint16_t elementNumber)
{
	ddiToElementNumber[ddi] = elementNumber;
}

bool ClientState::has_element_number_for_ddi(isobus::DataDescriptionIndex ddi) const
{
	return ddiToElementNumber.find(ddi) != ddiToElementNumber.end();
}

bool ClientState::is_element_or_parent_off(std::uint16_t elementNumber) const
{
	// Called per section from the heartbeat and the VT section map, so this only walks the cached
	// parent map; searching the pool here cost milliseconds per call on large pools (#100).
	return element_hierarchy::any_in_chain(elementParents, elementNumber, [this](std::uint16_t element) {
		bool elementWorkState;
		return try_get_element_work_state(element, elementWorkState) && !elementWorkState;
	});
}

void ClientState::build_element_hierarchy()
{
	elementParents = element_hierarchy::build_parent_map(pool);
}

void ClientState::set_element_work_state(std::uint16_t elementNumber, bool isWorking)
{
	elementWorkStates[elementNumber] = isWorking;
}

bool ClientState::try_get_element_work_state(std::uint16_t elementNumber, bool &isWorking) const
{
	auto it = elementWorkStates.find(elementNumber);
	if (it != elementWorkStates.end())
	{
		isWorking = it->second;
		return true;
	}
	return false;
}

void ClientState::clear_rate_elements()
{
	rateElements.clear();
}

void ClientState::add_rate_element(const RateElement &element)
{
	rateElements.push_back(element);
}

const std::vector<RateElement> &ClientState::get_rate_elements() const
{
	return rateElements;
}

void ClientState::set_rate_element_channel(std::size_t index, std::uint8_t channel)
{
	if (index < rateElements.size())
	{
		rateElements[index].channel = channel;
	}
}

std::vector<RateElement> &ClientState::get_rate_elements_mutable()
{
	return rateElements;
}

void ClientState::store_rate_value(std::uint16_t elementNumber, std::uint16_t ddi, std::int32_t value, std::uint32_t nowMs)
{
	for (auto &element : rateElements)
	{
		if (element.actualDdi == ddi && element.elementNumber == elementNumber)
		{
			element.actualValue = value;
			element.actualMs = nowMs;
		}
		if (element.totalDdi == ddi && element.totalElement == elementNumber)
		{
			element.totalValue = value; // a parent's total is kept by every rate element under it
			element.totalMs = nowMs;
		}
	}
}

MyTCServer::MyTCServer(std::shared_ptr<isobus::InternalControlFunction> internalControlFunction,
                       isobus::TaskControllerServer::TaskControllerVersion version) :
  TaskControllerServer(internalControlFunction,
                       1, // AOG limits to 1 boom
                       64, // AOG limits to 16 sections of unique width but can be 64 by using zones
                       64, // 64 channels for position based control
                       isobus::TaskControllerOptions()
                         .with_implement_section_control(), // We support section control
                       version)
{
}

bool MyTCServer::activate_object_pool(std::shared_ptr<isobus::ControlFunction> partnerCF, ObjectPoolActivationError &activationError, ObjectPoolErrorCodes &objectPoolError, std::uint16_t &parentObjectIDOfFaultyObject, std::uint16_t &faultyObjectID)
{
	std::lock_guard<std::recursive_mutex> lock(clientsMutex);

	// Default to "no error" / "no faulty object" up front, so every early-return path
	// below only needs to override what's actually wrong, per the meaning documented on
	// TaskControllerServer::activate_object_pool() in the base class.
	activationError = ObjectPoolActivationError::NoErrors;
	objectPoolError = ObjectPoolErrorCodes::NoErrors;
	parentObjectIDOfFaultyObject = isobus::NULL_OBJECT_ID;
	faultyObjectID = isobus::NULL_OBJECT_ID;

	log("TC Server") << "Client " << partnerCF->get_NAME().get_full_name() << " requesting object pool activation" << std::endl;
	// Safety check to make sure partnerCF has uploaded a DDOP
	if (uploadedPools.find(partnerCF) == uploadedPools.end())
	{
		// Not a DDOP content problem (there's no DDOP to have one) — this is a client
		// requesting activation without ever uploading, which isn't covered by a more
		// specific error code.
		activationError = ObjectPoolActivationError::AnyOtherError;
		return false;
	}

	// Initialize a new client state
	auto state = ClientState();
	// state.get_pool().set_task_controller_compatibility_level(get_active_client(partnerCF)->reportedVersion);
	state.get_pool().set_task_controller_compatibility_level(static_cast<std::uint8_t>(TaskControllerVersion::SecondEditionDraft));

	bool deserialized = false;
	std::vector<std::uint8_t> receivedPool;
	std::vector<std::size_t> receivedChunkSizes;
	while (!uploadedPools[partnerCF].empty())
	{
		auto binaryPool = uploadedPools[partnerCF].front();
		uploadedPools[partnerCF].pop();
		receivedPool.insert(receivedPool.end(), binaryPool.begin(), binaryPool.end());
		receivedChunkSizes.push_back(binaryPool.size());
		deserialized = state.get_pool().deserialize_binary_object_pool(binaryPool.data(), static_cast<std::uint32_t>(binaryPool.size()), partnerCF->get_NAME());
	}
	save_received_ddop(partnerCF->get_NAME().get_full_name(), receivedPool, receivedChunkSizes);
	if (deserialized)
	{
		log() << "Successfully deserialized device descriptor object pool." << std::endl;
		state.build_element_hierarchy();

		// Save to NVM
		std::shared_ptr<isobus::task_controller_object::DeviceObject> deviceObject;
		for (std::uint16_t i = 0; i < state.get_pool().size(); i++)
		{
			auto object = state.get_pool().get_object_by_index(i);
			if (object && object->get_object_type() == isobus::task_controller_object::ObjectTypes::Device)
			{
				deviceObject = std::static_pointer_cast<isobus::task_controller_object::DeviceObject>(object);
				break;
			}
		}

		if (!deviceObject)
		{
			// A spec-compliant pool always has exactly one Device object at its root.
			// If it's missing — a malformed pool, or a multi-chunk transfer that didn't
			// concatenate correctly upstream — reject the activation instead of crashing
			// on the dereference below. There's no single faulty object ID to point at
			// here (the problem is an absence, not a bad object), so parent/faulty stay
			// NULL_OBJECT_ID.
			log("TC Server") << "Client " << partnerCF->get_NAME().get_full_name()
			                 << " activation REJECTED: deserialized pool (" << state.get_pool().size()
			                 << " objects) has no Device object." << std::endl;
			activationError = ObjectPoolActivationError::ThereAreErrorsInTheDDOP;
			objectPoolError = ObjectPoolErrorCodes::UnknownObjectReference;
			return false;
		}

		auto labelBytes = deviceObject->get_localization_label();
		std::string label(reinterpret_cast<const char *>(labelBytes.data()), labelBytes.size());
		// trim at first non-printable character (control chars, DEL, etc.)
		auto it = std::find_if(label.begin(), label.end(), [](unsigned char c) { return c < 0x20 || c >= 0x7F; });
		label.erase(it, label.end());

		auto fileName = std::to_string(partnerCF->get_NAME().get_full_name()) + "/" + sanitize_filename(label) + ".ddop";
		std::vector<std::uint8_t> binaryPool;
		if (state.get_pool().generate_binary_object_pool(binaryPool))
		{
			std::ofstream outFile(Settings::get_filename_path(fileName), std::ios::binary);
			if (outFile.is_open())
			{
				outFile.write(reinterpret_cast<const char *>(binaryPool.data()), binaryPool.size());
				outFile.close();
				log() << "Saved DDOP to file: " << fileName << std::endl;
			}
			else
			{
				log() << "Unable to save DDOP to NVM. (Failed to open file) file: " << fileName << std::endl;
			}
		}
		else
		{
			log() << "Unable to save DDOP to NVM. (Failed to generate binary object pool)" << std::endl;
		}

		auto implement = isobus::DeviceDescriptorObjectPoolHelper::get_implement_geometry(state.get_pool());
		std::uint8_t numberOfSections = 0;

		// Build a flat list of section element numbers in the same order as the geometry enumeration
		std::vector<std::uint16_t> sectionElementNumbers;

		// One row per section, in the same order as sectionElementNumbers (the row index is AOG's section index)
		struct GeometryRow
		{
			std::uint16_t elementNumber;
			std::uint16_t boomElement;
			std::string subBoomElement; // "-" for sections attached directly to a boom
			isobus::DeviceDescriptorObjectPoolHelper::Section section;
		};
		std::vector<GeometryRow> geometryRows;

		for (const auto &boom : implement.booms)
		{
			for (const auto &subBoom : boom.subBooms)
			{
				for (const auto &section : subBoom.sections)
				{
					numberOfSections++;
					sectionElementNumbers.push_back(section.elementNumber);
					geometryRows.push_back({ section.elementNumber, boom.elementNumber, std::to_string(subBoom.elementNumber), section });
				}
			}
			for (const auto &section : boom.sections)
			{
				numberOfSections++;
				sectionElementNumbers.push_back(section.elementNumber);
				geometryRows.push_back({ section.elementNumber, boom.elementNumber, "-", section });
			}
		}

		// Values missing from the DDOP are shown as "-" rather than a misleading 0
		auto mm = [](const isobus::DeviceDescriptorObjectPoolHelper::ObjectPoolValue &value) {
			return value.exists() ? std::to_string(value.get()) : std::string("-");
		};
		log() << "Implement geometry: " << implement.booms.size() << " boom(s), " << static_cast<int>(numberOfSections) << " section(s)" << std::endl;
		if (!geometryRows.empty())
		{
			auto printRow = [](const std::string &idx, const std::string &element, const std::string &boom, const std::string &subBoom, const std::string &x, const std::string &y, const std::string &z, const std::string &width) {
				log() << std::right << std::setw(4) << idx << "  " << std::setw(7) << element << "  " << std::setw(4) << boom << "  " << std::setw(7) << subBoom
				      << "  " << std::setw(7) << x << "  " << std::setw(7) << y << "  " << std::setw(7) << z << "  " << std::setw(10) << width << std::endl;
			};
			printRow("Idx", "Element", "Boom", "SubBoom", "X (mm)", "Y (mm)", "Z (mm)", "Width (mm)");
			printRow("----", "-------", "----", "-------", "-------", "-------", "-------", "----------");
			for (std::size_t i = 0; i < geometryRows.size(); i++)
			{
				const auto &row = geometryRows[i];
				printRow(std::to_string(i), std::to_string(row.elementNumber), std::to_string(row.boomElement), row.subBoomElement, mm(row.section.xOffset_mm), mm(row.section.yOffset_mm), mm(row.section.zOffset_mm), mm(row.section.width_mm));
			}
		}
		state.set_number_of_sections(numberOfSections);

		// Map each section index to its element number from the geometry
		for (std::uint8_t i = 0; i < numberOfSections && i < sectionElementNumbers.size(); i++)
		{
			state.set_element_number_for_section(i, sectionElementNumbers[i]);
		}

		// Scan the DDOP to determine which section control method the device supports
		bool hasCondensedSetpoint = false; // Modern: DDI 290+ (paired with DDI 289 for global work state)
		bool hasSettableCondensedActual = false; // Old: DDI 161+ settable
		bool hasSettableActualWorkState = false; // Oldest: DDI 141 per-element settable

		for (std::uint32_t i = 0; i < state.get_pool().size(); i++)
		{
			auto object = state.get_pool().get_object_by_index(i);
			if (object && object->get_object_type() == isobus::task_controller_object::ObjectTypes::DeviceProcessData)
			{
				auto processDataObject = std::dynamic_pointer_cast<isobus::task_controller_object::DeviceProcessDataObject>(object);
				auto ddi = processDataObject->get_ddi();

				if (ddi >= static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SetpointCondensedWorkState1_16) &&
				    ddi <= static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SetpointCondensedWorkState241_256))
				{
					hasCondensedSetpoint = true;
				}
				if (ddi >= static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState1_16) &&
				    ddi <= static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState241_256) &&
				    processDataObject->has_property(isobus::task_controller_object::DeviceProcessDataObject::PropertiesBit::Settable))
				{
					hasSettableCondensedActual = true;
				}
				if (ddi == static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualWorkState) &&
				    processDataObject->has_property(isobus::task_controller_object::DeviceProcessDataObject::PropertiesBit::Settable))
				{
					hasSettableActualWorkState = true;
				}
			}
		}

		// Announce and configure the section control method (hierarchy: Modern > Old > Oldest)
		if (hasCondensedSetpoint)
		{
			// Modern: condensed setpoint DDI 290+ (always paired with DDI 289 for global work state)
			log("TC Server") << "Attempting Section Control via: DDI 290 (SetpointCondensedWorkState) + DDI 289 (SetpointWorkState)"
			                 << " for " << static_cast<int>(numberOfSections) << " sections." << std::endl;
		}
		else if (hasSettableCondensedActual)
		{
			// Old: settable condensed actual DDI 161+
			log("TC Server") << "Attempting Section Control via: DDI 161 (ActualCondensedWorkState, settable)"
			                 << " for " << static_cast<int>(numberOfSections) << " sections." << std::endl;
		}
		else if (hasSettableActualWorkState)
		{
			// Oldest: per-element settable DDI 141
			state.set_uses_per_element_control(true);
			state.set_per_element_setpoint_ddi(static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualWorkState));
			log("TC Server") << "Attempting Section Control via: DDI 141 (ActualWorkState, settable per-element)"
			                 << " for " << static_cast<int>(numberOfSections) << " sections." << std::endl;
			for (std::uint8_t i = 0; i < numberOfSections; i++)
			{
				log() << "  Section " << static_cast<int>(i) << " -> element " << sectionElementNumbers[i] << std::endl;
			}
		}
		else
		{
			log("TC Server") << "WARNING: No supported section control method detected! "
			                 << "Device has no DDI 290, 161 (settable), or 141 (settable)." << std::endl;
		}
	}
	else
	{
		log() << "Failed to deserialize device descriptor object pool." << std::endl;
		activationError = ObjectPoolActivationError::ThereAreErrorsInTheDDOP;
		objectPoolError = ObjectPoolErrorCodes::AnyOtherError;
		return false;
	}

	clients[partnerCF] = state;
	log("TC Server") << "Client " << partnerCF->get_NAME().get_full_name() << " registered successfully with "
	                 << static_cast<int>(state.get_number_of_sections()) << " sections." << std::endl;
	return true;
}

bool MyTCServer::change_designator(std::shared_ptr<isobus::ControlFunction>, std::uint16_t, const std::vector<std::uint8_t> &)
{
	return true;
}

bool MyTCServer::deactivate_object_pool(std::shared_ptr<isobus::ControlFunction> partnerCF)
{
	std::lock_guard<std::recursive_mutex> lock(clientsMutex);
	clients.erase(partnerCF);
	uploadedPools.erase(partnerCF);
	return true;
}

bool MyTCServer::delete_device_descriptor_object_pool(std::shared_ptr<isobus::ControlFunction> partnerCF, ObjectPoolDeletionErrors &)
{
	std::lock_guard<std::recursive_mutex> lock(clientsMutex);
	clients.erase(partnerCF);
	uploadedPools.erase(partnerCF);
	return true;
}

bool MyTCServer::get_is_stored_device_descriptor_object_pool_by_structure_label(std::shared_ptr<isobus::ControlFunction>, const std::vector<std::uint8_t> &, const std::vector<std::uint8_t> &)
{
	return false;
}

bool MyTCServer::get_is_stored_device_descriptor_object_pool_by_localization_label(std::shared_ptr<isobus::ControlFunction>, const std::array<std::uint8_t, 7> &)
{
	return false;
}

bool MyTCServer::get_is_enough_memory_available(std::uint32_t)
{
	return true;
}

void MyTCServer::identify_task_controller(std::uint8_t tcNumber)
{
	// ISO 11783-10 B.5.4/B.5.5: Identify Task Controller message
	// When this is called, the TC shall display its TC number for 3 seconds
	// TC Number = Function Instance + 1 (range 1-32)
	//
	// Since this is a console application without GUI, we log to console
	// In a GUI application, this would display the TC number visually
	log() << "========================================" << std::endl;
	log() << "=== TC NUMBER " << static_cast<int>(tcNumber) << " IDENTIFIED ===" << std::endl;
	log() << "========================================" << std::endl;
}

void MyTCServer::on_client_timeout(std::shared_ptr<isobus::ControlFunction> partner)
{
	std::lock_guard<std::recursive_mutex> lock(clientsMutex);
	// Cleanup the client state
	log("TC Server") << "Client " << partner->get_NAME().get_full_name() << " has timed out!" << std::endl;
	clients.erase(partner);
}

void MyTCServer::on_client_version_received(std::shared_ptr<isobus::ControlFunction> clientControlFunction, std::uint8_t version)
{
	log("TC Server") << "Client " << clientControlFunction->get_NAME().get_full_name()
	                 << " reported TC version " << static_cast<int>(version) << std::endl;
}

void MyTCServer::on_process_data_acknowledge(std::shared_ptr<isobus::ControlFunction> partner,
                                             std::uint16_t dataDescriptionIndex,
                                             std::uint16_t elementNumber,
                                             std::uint8_t errorCodesFromClient,
                                             ProcessDataCommands processDataCommand)
{
	// This callback lets you know when a client sends a process data acknowledge (PDACK) message to you
	log() << "Received process data acknowledge from client " << int(partner->get_address()) << " for DDI " << dataDescriptionIndex << " element " << elementNumber << " with error codes " << std::bitset<8>(errorCodesFromClient) << " and command " << static_cast<int>(processDataCommand) << std::endl;
}

bool MyTCServer::on_value_command(std::shared_ptr<isobus::ControlFunction> partner,
                                  std::uint16_t dataDescriptionIndex,
                                  std::uint16_t elementNumber,
                                  std::int32_t processDataValue,
                                  std::uint8_t &errorCodes)
{
	std::lock_guard<std::recursive_mutex> lock(clientsMutex);
	switch (dataDescriptionIndex)
	{
		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState1_16):
		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState17_32):
		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState33_48):
		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState49_64):
		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState65_80):
		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState81_96):
		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState97_112):
		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState113_128):
		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState129_144):
		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState145_160):
		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState161_176):
		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState177_192):
		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState193_208):
		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState209_224):
		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState225_240):
		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState241_256):
		{
			std::uint8_t sectionIndexOffset = NUMBER_SECTIONS_PER_CONDENSED_MESSAGE * static_cast<std::uint8_t>(dataDescriptionIndex - static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState1_16));

			for (std::uint_fast8_t i = 0; i < NUMBER_SECTIONS_PER_CONDENSED_MESSAGE; i++)
			{
				std::uint8_t sectionState = ((processDataValue >> (2 * i)) & 0x03);
				clients[partner].set_section_actual_state(i + sectionIndexOffset, sectionState);
				clients[partner].set_element_number_for_section(i + sectionIndexOffset, elementNumber);
			}
		}
		break;

		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SectionControlState):
		{
			clients[partner].set_section_control_enabled(processDataValue == 1);
		}
		break;

		case static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualWorkState):
		{
			// Store the work state per element (used for parent-off checks)
			clients[partner].set_element_work_state(elementNumber, processDataValue == 1);

			// For legacy per-element devices only: propagate to section actual states
			// so the heartbeat (PGN 0xF0) can report them to AOG.
			// Modern implements may report both DDI 141 and condensed DDIs (161/290)
			// on the same element — only overwrite section states when per-element
			// control is the active section control method.
			if (clients[partner].uses_per_element_control())
			{
				std::uint8_t sectionIndex;
				if (clients[partner].try_get_section_for_element(elementNumber, sectionIndex))
				{
					clients[partner].set_section_actual_state(sectionIndex, (processDataValue == 1) ? SectionState::ON : SectionState::OFF);
				}
			}
		}
	}

	// prescription control state / mode and default rates of the rate elements (step 3)
	on_rx_value(clients[partner], dataDescriptionIndex, elementNumber, processDataValue);

	// actual rates and totals of the rate elements, for RateController (PGN 0xF5)
	clients[partner].store_rate_value(elementNumber, dataDescriptionIndex, processDataValue, std::max<std::uint32_t>(isobus::SystemTiming::get_timestamp_ms(), 1));

	return true;
}

bool MyTCServer::store_device_descriptor_object_pool(std::shared_ptr<isobus::ControlFunction> partnerCF, const std::vector<std::uint8_t> &binaryPool, bool appendToPool)
{
	std::lock_guard<std::recursive_mutex> lock(clientsMutex);
	log("TC Server") << "Client " << partnerCF->get_NAME().get_full_name() << " requesting object pool transfer of " << binaryPool.size() << " bytes" << std::endl;
	if (uploadedPools.find(partnerCF) == uploadedPools.end())
	{
		uploadedPools[partnerCF] = std::queue<std::vector<std::uint8_t>>();
	}
	uploadedPools[partnerCF].push(binaryPool);
	return true;
}

std::map<std::shared_ptr<isobus::ControlFunction>, ClientState> MyTCServer::get_clients()
{
	std::lock_guard<std::recursive_mutex> lock(clientsMutex);
	return clients; // copy, taken while locked — see the declaration's comment
}

void MyTCServer::request_measurement_commands()
{
	std::lock_guard<std::recursive_mutex> lock(clientsMutex);
	for (auto &client : clients)
	{
		// Skip clients with 0 sections (e.g. tractors) - sending measurement commands to a tractor ECU can cause unexpected behavior
		if (!client.second.are_measurement_commands_sent() && client.second.get_number_of_sections() > 0)
		{
			// Find all actual (condensed) work state DDIs and request them to trigger "On Change" and "Time Interval"
			for (std::uint32_t i = 0; i < client.second.get_pool().size(); i++)
			{
				auto object = client.second.get_pool().get_object_by_index(i);
				if (object && object->get_object_type() == isobus::task_controller_object::ObjectTypes::DeviceProcessData)
				{
					auto processDataObject = std::dynamic_pointer_cast<isobus::task_controller_object::DeviceProcessDataObject>(object);
					if (processDataObject->get_ddi() == static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualWorkState) ||
					    (processDataObject->get_ddi() >= static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState1_16) &&
					     processDataObject->get_ddi() <= static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState241_256)) ||
					    (processDataObject->get_ddi() >= static_cast<std::uint16_t>(isobus::DataDescriptionIndex::CondensedSectionOverrideState1_16) &&
					     processDataObject->get_ddi() <= static_cast<std::uint16_t>(isobus::DataDescriptionIndex::CondensedSectionOverrideState241_256)))
					{
						// Loop over all objects to find the elements that are the parents of the actual condensed work state objects
						for (std::uint32_t j = 0; j < client.second.get_pool().size(); j++)
						{
							auto parentObject = client.second.get_pool().get_object_by_index(j);
							if (parentObject && parentObject->get_object_type() == isobus::task_controller_object::ObjectTypes::DeviceElement)
							{
								auto elementObject = std::dynamic_pointer_cast<isobus::task_controller_object::DeviceElementObject>(parentObject);
								for (std::uint16_t elementObjectChild : elementObject->get_child_object_ids())
								{
									if (elementObjectChild == processDataObject->get_object_id())
									{
										// TODO: This is a bit of a hack, but it works for now
										client.second.set_element_number_for_ddi(static_cast<isobus::DataDescriptionIndex>(processDataObject->get_ddi()), elementObject->get_element_number());
										const auto &entryB = isobus::DataDictionary::get_entry(processDataObject->get_ddi());
										log() << "Mapped DDI " << processDataObject->get_ddi() << " (" << entryB.to_string() << ") to element "
										      << elementObject->get_element_number() << std::endl;

										if (processDataObject->has_trigger_method(isobus::task_controller_object::DeviceProcessDataObject::AvailableTriggerMethods::OnChange))
										{
											send_change_threshold_measurement_command(client.first, processDataObject->get_ddi(), elementObject->get_element_number(), 1);
											log() << "Subscribed (OnChange) to DDI " << processDataObject->get_ddi() << " (" << entryB.to_string() << ") for element "
											      << elementObject->get_element_number() << std::endl;
										}
										if (processDataObject->has_trigger_method(isobus::task_controller_object::DeviceProcessDataObject::AvailableTriggerMethods::TimeInterval))
										{
											send_time_interval_measurement_command(client.first, processDataObject->get_ddi(), elementObject->get_element_number(), 1000);
										}
									}
								}
							}
						}
					}
				}
			}

			// Find all section control state DDIs and request them to trigger "On Change"
			for (std::uint32_t i = 0; i < client.second.get_pool().size(); i++)
			{
				auto object = client.second.get_pool().get_object_by_index(i);
				if (object && object->get_object_type() == isobus::task_controller_object::ObjectTypes::DeviceProcessData)
				{
					auto processDataObject = std::dynamic_pointer_cast<isobus::task_controller_object::DeviceProcessDataObject>(object);
					if (processDataObject->get_ddi() == static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SectionControlState) ||
					    processDataObject->get_ddi() == static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SetpointWorkState) ||
					    (processDataObject->get_ddi() >= static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SetpointCondensedWorkState1_16) &&
					     processDataObject->get_ddi() <= static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SetpointCondensedWorkState241_256)))
					{
						// Loop over all objects to find the elements that are the parents of the section control state objects
						for (std::uint32_t j = 0; j < client.second.get_pool().size(); j++)
						{
							auto parentObject = client.second.get_pool().get_object_by_index(j);
							if (parentObject && parentObject->get_object_type() == isobus::task_controller_object::ObjectTypes::DeviceElement)
							{
								auto elementObject = std::dynamic_pointer_cast<isobus::task_controller_object::DeviceElementObject>(parentObject);
								for (std::uint16_t elementObjectChild : elementObject->get_child_object_ids())
								{
									if (elementObjectChild == processDataObject->get_object_id())
									{
										// TODO: This is a bit of a hack, but it works for now
										client.second.set_element_number_for_ddi(static_cast<isobus::DataDescriptionIndex>(processDataObject->get_ddi()), elementObject->get_element_number());
										const auto &entryB = isobus::DataDictionary::get_entry(processDataObject->get_ddi());

										if (processDataObject->has_trigger_method(isobus::task_controller_object::DeviceProcessDataObject::AvailableTriggerMethods::OnChange))
										{
											send_change_threshold_measurement_command(client.first, processDataObject->get_ddi(), elementObject->get_element_number(), 1);
											log() << "Subscribed (OnChange) to DDI " << processDataObject->get_ddi() << " (" << entryB.to_string() << ") for element "
											      << elementObject->get_element_number() << std::endl;
										}
										else
										{
											log() << "Mapped (no OnChange) DDI " << processDataObject->get_ddi() << " (" << entryB.to_string() << ") to element "
											      << elementObject->get_element_number() << std::endl;
										}
									}
								}
							}
						}
					}
				}
			}

			// Map GNSS quality (DDI 514) so send_gnss_quality() knows which element to address.
			// Mapping only: this is a value the TC pushes to the implement, not one to subscribe to.
			for (std::uint32_t i = 0; i < client.second.get_pool().size(); i++)
			{
				auto object = client.second.get_pool().get_object_by_index(i);
				if (!object || object->get_object_type() != isobus::task_controller_object::ObjectTypes::DeviceProcessData)
				{
					continue;
				}

				auto processDataObject = std::dynamic_pointer_cast<isobus::task_controller_object::DeviceProcessDataObject>(object);
				if (!processDataObject || processDataObject->get_ddi() != static_cast<std::uint16_t>(isobus::DataDescriptionIndex::GNSSQuality))
				{
					continue;
				}

				if (!processDataObject->has_property(isobus::task_controller_object::DeviceProcessDataObject::PropertiesBit::Settable))
				{
					log() << "DDI " << processDataObject->get_ddi() << " (GNSS Quality) is declared but not settable, so it is not sent" << std::endl;
					continue;
				}

				std::uint16_t elementNumber = 0;
				if (find_owning_element_number(client.second.get_pool(), *processDataObject, elementNumber))
				{
					client.second.set_element_number_for_ddi(isobus::DataDescriptionIndex::GNSSQuality, elementNumber);
					log() << "Mapped DDI " << processDataObject->get_ddi() << " (GNSS Quality) to element "
					      << elementNumber << std::endl;
				}
			}

			// List the settable rate DDIs (1/6/11); match_rate_elements() puts each on the channel of the same type.
			// Interim until channels can be assigned per element (Group2_Step1_Channels_Design.md, RateController).
			client.second.clear_rate_elements();
			for (std::uint32_t i = 0; i < client.second.get_pool().size(); i++)
			{
				auto object = client.second.get_pool().get_object_by_index(i);
				if (!object || object->get_object_type() != isobus::task_controller_object::ObjectTypes::DeviceProcessData)
				{
					continue;
				}

				auto processDataObject = std::dynamic_pointer_cast<isobus::task_controller_object::DeviceProcessDataObject>(object);
				if (!processDataObject || !is_rate_setpoint_ddi(processDataObject->get_ddi()))
				{
					continue;
				}

				const auto &entry = isobus::DataDictionary::get_entry(processDataObject->get_ddi());
				if (!processDataObject->has_property(isobus::task_controller_object::DeviceProcessDataObject::PropertiesBit::Settable))
				{
					log("Rate") << "DDI " << processDataObject->get_ddi() << " (" << entry.to_string() << ") is declared but not settable, so no rate is sent to it" << std::endl;
					continue;
				}

				std::uint16_t elementNumber = 0;
				if (find_owning_element_number(client.second.get_pool(), *processDataObject, elementNumber))
				{
					const std::string designator = element_designator(client.second.get_pool(), elementNumber);
					client.second.add_rate_element({ processDataObject->get_ddi(), elementNumber, 0, designator });
					log("Rate") << "Rate element " << elementNumber << " \"" << designator << "\": DDI " << processDataObject->get_ddi()
					            << " (" << entry.to_string() << ")" << std::endl;
				}
			}
			for (auto &element : client.second.get_rate_elements_mutable())
			{
				subscribe_rate_actuals(client.first, client.second, element);
				find_rx_ddis(client.first, client.second, element);
			}
			match_rate_elements(client.first, client.second, true);

			log() << "Measurement commands sent." << std::endl;
			client.second.mark_measurement_commands_sent();

			// Switch the implement to automatic shortly after it connects, so AOG's SC button
			// does not need pressing on every connect. With the setting off, send nothing, as before.
			if (sectionControlOnConnect)
			{
				client.second.schedule_initial_section_control(isobus::SystemTiming::get_timestamp_ms() + INITIAL_SECTION_CONTROL_DELAY_MS);
			}
		}
		if (client.second.take_due_initial_section_control(isobus::SystemTiming::get_timestamp_ms()))
		{
			log("TC") << "Applying initial section control state: " << (desiredSectionControlEnabled ? "enabled" : "disabled") << std::endl;
			apply_section_control_state(client.first, client.second, desiredSectionControlEnabled);
		}
	}
}

void MyTCServer::update_section_states(std::vector<bool> &sectionStates)
{
	std::lock_guard<std::recursive_mutex> lock(clientsMutex);
	aogSectionDemand = sectionStates;
	lastSectionDemandMs = isobus::SystemTiming::get_timestamp_ms();
	if (0 == lastSectionDemandMs)
	{
		lastSectionDemandMs = 1; // 0 means "never"
	}
	for (auto &client : clients)
	{
		auto &state = client.second;
		if (!state.is_section_control_enabled())
		{
			// According to standard, the section setpoint states should only be sent when in auto mode
			continue;
		}

		// Skip clients that don't have any sections configured (e.g., tractors or other non-implement devices)
		if (state.get_number_of_sections() == 0)
		{
			continue;
		}

		bool requiresUpdate = false;
		for (std::uint8_t i = 0; i < state.get_number_of_sections(); i++)
		{
			if ((i % NUMBER_SECTIONS_PER_CONDENSED_MESSAGE == 0) && requiresUpdate)
			{
				// Send the previous 16 sections
				std::uint8_t ddiOffset = (i / NUMBER_SECTIONS_PER_CONDENSED_MESSAGE) - 1;
				send_section_setpoint_states(client.first, ddiOffset);
				requiresUpdate = false;
			}

			if (i < sectionStates.size())
			{
				if (sectionStates[i] != (state.get_section_setpoint_state(i) == SectionState::ON))
				{
					state.set_section_setpoint_state(i, sectionStates[i] ? SectionState::ON : SectionState::OFF);
					requiresUpdate = true;
				}
			}
		}
		if (requiresUpdate)
		{
			std::uint8_t ddiOffset = (state.get_number_of_sections() - 1) / NUMBER_SECTIONS_PER_CONDENSED_MESSAGE;
			send_section_setpoint_states(client.first, ddiOffset);
		}
	}
}

void MyTCServer::update_section_control_enabled(bool enabled)
{
	std::lock_guard<std::recursive_mutex> lock(clientsMutex);
	desiredSectionControlEnabled = enabled;
	for (auto &client : clients)
	{
		apply_section_control_state(client.first, client.second, enabled);
	}
}

void MyTCServer::set_section_control_on_connect(bool enabled)
{
	std::lock_guard<std::recursive_mutex> lock(clientsMutex);
	sectionControlOnConnect = enabled;
	desiredSectionControlEnabled = enabled;
}

void MyTCServer::apply_section_control_state(std::shared_ptr<isobus::ControlFunction> client, ClientState &state, bool enabled)
{
	// Always update the local flag
	state.set_section_control_enabled(enabled);

	// Skip clients that don't have any sections configured (e.g., tractors or other non-implement devices)
	if (state.get_number_of_sections() == 0)
	{
		return;
	}

	// Only send ISOBUS command to clients that support SectionControlState DDI
	if (state.has_element_number_for_ddi(isobus::DataDescriptionIndex::SectionControlState))
	{
		send_section_control_state(client, enabled);
	}

	if (enabled)
	{
		// ISO 11783-10 F.3.5.3 b): after switching to automatic, send the setpoint. update_section_states()
		// only sends changes, so an all-off demand (AOG's Auto off after a field change) would never reach
		// the implement and it would keep running on its own state. A demand older than
		// SECTION_DEMAND_TIMEOUT_MS (no field open) counts as all off.
		const bool demandCurrent = (0 != lastSectionDemandMs) && !isobus::SystemTiming::time_expired_ms(lastSectionDemandMs, SECTION_DEMAND_TIMEOUT_MS);
		for (std::uint8_t i = 0; i < state.get_number_of_sections(); i++)
		{
			const bool on = demandCurrent && (i < aogSectionDemand.size()) && aogSectionDemand[i];
			state.set_section_setpoint_state(i, on ? SectionState::ON : SectionState::OFF);
		}
		send_all_section_setpoint_states(client);
	}
}

void MyTCServer::send_all_section_setpoint_states(std::shared_ptr<isobus::ControlFunction> client)
{
	std::lock_guard<std::recursive_mutex> lock(clientsMutex);
	const std::uint8_t numberOfSections = clients[client].get_number_of_sections();
	for (std::uint8_t block = 0; block * NUMBER_SECTIONS_PER_CONDENSED_MESSAGE < numberOfSections; block++)
	{
		send_section_setpoint_states(client, block, true);
	}
}

bool MyTCServer::is_rate_setpoint_ddi(std::uint16_t ddi)
{
	return ddi == static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SetpointVolumePerAreaApplicationRate) ||
	  ddi == static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SetpointMassPerAreaApplicationRate) ||
	  ddi == static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SetpointCountPerAreaApplicationRate);
}

void MyTCServer::send_rate_to_elements(std::uint8_t channel, std::uint16_t ddi, std::int32_t value)
{
	for (auto &client : clients)
	{
		for (const auto &element : client.second.get_rate_elements())
		{
			if (element.channel == channel && element.ddi == ddi &&
			    (element.rxState == RxState::Starting || element.rxState == RxState::Active))
			{
				send_set_value(client.first, ddi, element.elementNumber, value);
			}
		}
	}
}

std::string MyTCServer::rate_element_text(const RateElement &element)
{
	return "Element " + std::to_string(element.elementNumber) + " \"" + element.designator + "\"";
}

void MyTCServer::send_rx_command(std::shared_ptr<isobus::ControlFunction> client, RateElement &element, const RateChannel *rate)
{
	constexpr std::uint16_t DDI_RX_STATE = 158; // Prescription Control State
	constexpr std::uint16_t DDI_RX_SOURCE = 287; // Setpoint Prescription Mode
	if (element.rxWanted == 1)
	{
		// ON in this order (step 3 § 5): 287 = 1 (TC rate), 158 = 1, then the setpoint straight after, without waiting
		if (element.has287)
		{
			send_set_value(client, DDI_RX_SOURCE, element.sourceElement, 1);
		}
		if (element.has158)
		{
			send_set_value(client, DDI_RX_STATE, element.rxElement, 1);
		}
		if (rate && rate->ddi == element.ddi && rate->lastReceivedMs != 0)
		{
			send_set_value(client, element.ddi, element.elementNumber, rate->value);
		}
	}
	else if (element.has158)
	{
		send_set_value(client, DDI_RX_STATE, element.rxElement, 0);
	}

	element.rxTries++;
	element.rxConfirming = element.has158;
	element.rxCommandMs = std::max<std::uint32_t>(isobus::SystemTiming::get_timestamp_ms(), 1);
	element.rxAsked = false;
}

void MyTCServer::start_rate_element(std::shared_ptr<isobus::ControlFunction> client, RateElement &element, const RateChannel &rate)
{
	element.noReply = false;
	element.rxTries = 0;
	element.rxWanted = 1;
	if (element.has158 && element.rxReported != 3) // 3 = not installed: as no 158
	{
		element.rxState = RxState::Starting;
		log("Rate") << rate_element_text(element) << ": asking for prescription control (287 = 1, 158 = 1)" << std::endl;
	}
	else
	{
		element.rxState = RxState::Active;
	}
	send_rx_command(client, element, &rate);
}

void MyTCServer::hand_back_rate_element(std::shared_ptr<isobus::ControlFunction> client, RateElement &element, RxState newState)
{
	element.rxState = newState;
	element.noReply = false;
	element.rxTries = 0;
	element.rxWanted = 0;
	if (element.has158 && element.rxReported != 3)
	{
		send_rx_command(client, element, nullptr); // no 0 setpoint: in manual the implement uses its own rate
	}
	else
	{
		element.rxConfirming = false;
		send_set_value(client, element.ddi, element.elementNumber, 0); // no 158 to hand back with: stop it
	}
}

void MyTCServer::update_rx_confirm(std::shared_ptr<isobus::ControlFunction> client, RateElement &element, std::uint32_t now)
{
	constexpr std::uint16_t DDI_RX_STATE = 158;
	if (!element.rxConfirming)
	{
		return;
	}

	if (!element.rxAsked)
	{
		if (isobus::SystemTiming::time_expired_ms(element.rxCommandMs, RX_CONFIRM_MS))
		{
			// a lost command causes no change, so the implement wouldn't report anything: ask for it
			send_request_value(client, DDI_RX_STATE, element.rxElement);
			element.rxAsked = true;
			element.rxAskedMs = std::max<std::uint32_t>(now, 1);
		}
	}
	else if (isobus::SystemTiming::time_expired_ms(element.rxAskedMs, RX_CONFIRM_MS))
	{
		if (element.rxTries < RX_TRIES)
		{
			const auto &rate = rateChannels[element.channel == 0 ? 0 : element.channel - 1];
			send_rx_command(client, element, element.channel == 0 ? nullptr : &rate); // not answered: resend
		}
		else
		{
			element.rxConfirming = false;
			element.noReply = true;
			log("Rate") << rate_element_text(element) << ": no reply to prescription control state" << std::endl;
			if (element.rxWanted == 1)
			{
				element.rxState = RxState::Refused;
				pendingRateAlerts.push_back(element.designator + ": no reply to prescription control");
			}
			// for OFF: stays OFF, setpoints stop anyway
		}
	}
}

void MyTCServer::on_rx_value(ClientState &state, std::uint16_t ddi, std::uint16_t elementNumber, std::int32_t value)
{
	constexpr std::uint16_t DDI_RX_STATE = 158;
	constexpr std::uint16_t DDI_RX_MODE = 288; // Actual Prescription Mode
	const std::uint32_t now = std::max<std::uint32_t>(isobus::SystemTiming::get_timestamp_ms(), 1);

	for (auto &element : state.get_rate_elements_mutable())
	{
		if (ddi == DDI_RX_MODE && element.has288 && element.modeElement == elementNumber)
		{
			element.modeReported = value;
		}

		if (element.defaultElement != 0 && element.defaultElement == elementNumber && ddi == element.ddi + 2) // 1 -> 3, 6 -> 8, 11 -> 13
		{
			element.hasDefault = true;
			element.defaultRate = value;
		}

		if (ddi != DDI_RX_STATE || !element.has158 || element.rxElement != elementNumber)
		{
			continue;
		}

		const std::int32_t before = element.rxReported;
		element.rxReported = value;
		element.rxReportedMs = now;
		element.noReply = false;
		if (element.rxConfirming && (value == element.rxWanted || element.rxWanted == 1))
		{
			element.rxConfirming = false; // answered (a refusal is an answer too)
		}

		if (value == 1 || value == 3)
		{
			if (element.rxState == RxState::Starting)
			{
				element.rxState = RxState::Active;
				log("Rate") << rate_element_text(element) << ": prescription control on" << std::endl;
			}
		}
		else if (element.rxState == RxState::Starting || element.rxState == RxState::Active)
		{
			// 0 or 2: the implement or its operator chose this. Never asked again by itself.
			const bool wasActive = element.rxState == RxState::Active;
			element.rxState = RxState::Refused;
			const std::string what = (value == 2) ? "reports a prescription control error" :
			  (wasActive ? "turned prescription control off" : "refused prescription control");
			log("Rate") << rate_element_text(element) << ": implement " << what << std::endl;
			pendingRateAlerts.push_back(element.designator + ": " + what);
		}
		else if (before != value)
		{
			log("Rate") << rate_element_text(element) << ": prescription control state " << value << std::endl;
		}
	}
}

void MyTCServer::find_rx_ddis(std::shared_ptr<isobus::ControlFunction> client, ClientState &state, RateElement &element)
{
	using Trigger = isobus::task_controller_object::DeviceProcessDataObject::AvailableTriggerMethods;
	constexpr std::uint16_t DDI_RX_STATE = 158;
	constexpr std::uint16_t DDI_RX_SOURCE = 287;
	constexpr std::uint16_t DDI_RX_MODE = 288;
	const std::uint16_t defaultDdi = element.ddi + 2; // 3 volume, 8 mass, 13 count per area
	auto &pool = state.get_pool();

	auto onChange = [&](isobus::task_controller_object::DeviceProcessDataObject &processData, std::uint16_t elementNumber) {
		if (processData.has_trigger_method(Trigger::OnChange))
		{
			send_change_threshold_measurement_command(client, processData.get_ddi(), elementNumber, 1);
		}
		else if (processData.has_trigger_method(Trigger::TimeInterval))
		{
			send_time_interval_measurement_command(client, processData.get_ddi(), elementNumber, 1000);
		}
		// else: the confirm requests it after each command
	};

	auto level = element_by_number(pool, element.elementNumber);
	for (int depth = 0; level && depth < 8; depth++)
	{
		const std::uint16_t number = level->get_element_number();
		if (!element.has158)
		{
			auto processData = child_process_data(pool, *level, DDI_RX_STATE);
			if (processData)
			{
				element.has158 = true;
				element.rxElement = number;
				onChange(*processData, number);
			}
		}
		if (!element.has287)
		{
			auto processData = child_process_data(pool, *level, DDI_RX_SOURCE);
			if (processData && processData->has_property(isobus::task_controller_object::DeviceProcessDataObject::PropertiesBit::Settable))
			{
				element.has287 = true;
				element.sourceElement = number;
			}
		}
		if (!element.has288)
		{
			auto processData = child_process_data(pool, *level, DDI_RX_MODE);
			if (processData)
			{
				element.has288 = true;
				element.modeElement = number;
				onChange(*processData, number);
			}
		}
		if (!element.hasDefault && element.defaultElement == 0)
		{
			// the default rate as a fixed property, or as process data the implement is asked for
			for (std::uint16_t childId : level->get_child_object_ids())
			{
				auto object = pool.get_object_by_id(childId);
				if (!element.hasDefault && object && object->get_object_type() == isobus::task_controller_object::ObjectTypes::DeviceProperty)
				{
					auto property = std::dynamic_pointer_cast<isobus::task_controller_object::DevicePropertyObject>(object);
					if (property && property->get_ddi() == defaultDdi)
					{
						element.hasDefault = true;
						element.defaultRate = property->get_value();
					}
				}
			}
			if (!element.hasDefault)
			{
				auto processData = child_process_data(pool, *level, defaultDdi);
				if (processData)
				{
					element.defaultElement = number;
					send_request_value(client, defaultDdi, number);
				}
			}
		}

		auto parent = pool.get_object_by_id(level->get_parent_object());
		level = (parent && parent->get_object_type() == isobus::task_controller_object::ObjectTypes::DeviceElement && parent != level) ?
		  std::dynamic_pointer_cast<isobus::task_controller_object::DeviceElementObject>(parent) :
		  nullptr;
	}

	log("Rate") << rate_element_text(element) << ": 158 " << (element.has158 ? "on element " + std::to_string(element.rxElement) : std::string("no"))
	            << ", 287 " << (element.has287 ? "on element " + std::to_string(element.sourceElement) : std::string("no"))
	            << ", 288 " << (element.has288 ? "on element " + std::to_string(element.modeElement) : std::string("no"))
	            << ", default rate " << (element.hasDefault ? std::to_string(element.defaultRate) : (element.defaultElement != 0 ? std::string("requested") : std::string("no")))
	            << std::endl;
}

void MyTCServer::update_rate_rx_request(std::uint8_t channel, std::int32_t value)
{
	if (channel < 1 || channel > MAX_RATE_CHANNELS)
	{
		return;
	}

	std::lock_guard<std::recursive_mutex> lock(clientsMutex);
	RateChannel &rate = rateChannels[channel - 1];
	rate.lastRcMs = std::max<std::uint32_t>(isobus::SystemTiming::get_timestamp_ms(), 1);
	rate.lost = false;

	if (value == 0)
	{
		// VR off: the implement's own rate. Repeating 0 is harmless; only a change acts.
		if (rate.vrOn || !rate.vrOffLogged)
		{
			log("Rate") << "Channel " << int(channel) << ": VR off, elements use their own rate (158 = 0)" << std::endl;
		}
		rate.vrOn = false;
		rate.vrOffLogged = true;
		for (auto &client : clients)
		{
			for (auto &element : client.second.get_rate_elements_mutable())
			{
				// also an element that is OFF but never commanded (just connected) and not known to be off
				const bool neverCommanded = element.rxState == RxState::Off && element.rxTries == 0 && element.has158 && element.rxReported != 0;
				if (element.channel == channel && (element.rxState != RxState::Off || neverCommanded))
				{
					hand_back_rate_element(client.first, element, RxState::Off);
				}
			}
		}
	}
	else if (value == 1)
	{
		// "Ask again": a fresh request for elements that refused or didn't reply
		log("Rate") << "Channel " << int(channel) << ": RateController asks again for prescription control" << std::endl;
		for (auto &client : clients)
		{
			for (auto &element : client.second.get_rate_elements_mutable())
			{
				if (element.channel == channel && (element.rxState == RxState::Refused || element.noReply))
				{
					start_rate_element(client.first, element, rate);
				}
			}
		}
	}
}

std::vector<std::string> MyTCServer::take_rate_alerts()
{
	std::lock_guard<std::recursive_mutex> lock(clientsMutex);
	std::vector<std::string> result;
	result.swap(pendingRateAlerts);
	return result;
}

void MyTCServer::update_rate_setpoint(std::uint8_t channel, std::uint16_t ddi, std::int32_t value)
{
	if (channel < 1 || channel > MAX_RATE_CHANNELS || !is_rate_setpoint_ddi(ddi))
	{
		return;
	}

	std::lock_guard<std::recursive_mutex> lock(clientsMutex);
	RateChannel &rate = rateChannels[channel - 1];
	const std::uint32_t now = std::max<std::uint32_t>(isobus::SystemTiming::get_timestamp_ms(), 1);
	value = std::max<std::int32_t>(value, 0);

	if (rate.ddi != 0 && rate.ddi != ddi && !rate.lost)
	{
		send_rate_to_elements(channel, rate.ddi, 0); // the channel changed DDI: stop the elements on the old one
	}

	const bool changed = (rate.lastReceivedMs == 0) || rate.lost || (rate.ddi != ddi) || (rate.value != value);
	if (rate.lost)
	{
		log("Rate") << "Channel " << int(channel) << ": RateController heard again, setpoints resumed" << std::endl;
	}
	if (!rate.vrOn && rate.lastReceivedMs != 0)
	{
		log("Rate") << "Channel " << int(channel) << ": VR on, RateController's rate" << std::endl;
	}
	const bool typeChanged = !rate.infoReceived && rate.ddi != ddi; // without channel info the setpoints give the type
	rate.ddi = ddi;
	rate.value = value;
	rate.lastReceivedMs = now;
	rate.lastRcMs = now;
	rate.lost = false;
	rate.vrOn = true;

	if (typeChanged)
	{
		match_rate_elements();
	}

	if (changed)
	{
		log("Rate") << "Channel " << int(channel) << ": DDI " << ddi << " setpoint " << value << std::endl;
		send_rate_to_elements(channel, ddi, value);
		rate.lastSentMs = now;
	}

	// OFF (VR was off, or a new implement / DDOP) and FALLBACK start a fresh request; REFUSED waits for "Ask again"
	for (auto &client : clients)
	{
		for (auto &element : client.second.get_rate_elements_mutable())
		{
			if (element.channel == channel && element.ddi == ddi &&
			    (element.rxState == RxState::Off || element.rxState == RxState::Fallback))
			{
				start_rate_element(client.first, element, rate);
			}
		}
	}
}

void MyTCServer::update_rate_setpoints()
{
	std::lock_guard<std::recursive_mutex> lock(clientsMutex);
	const std::uint32_t now = isobus::SystemTiming::get_timestamp_ms();
	for (std::uint8_t i = 0; i < MAX_RATE_CHANNELS; i++)
	{
		RateChannel &rate = rateChannels[i];
		const std::uint8_t channel = i + 1;

		// confirm 158 after each command, whatever RC is doing
		for (auto &client : clients)
		{
			for (auto &element : client.second.get_rate_elements_mutable())
			{
				if (element.channel == channel)
				{
					update_rx_confirm(client.first, element, now);
				}
			}
		}

		if (rate.lastRcMs == 0 || rate.lost)
		{
			continue;
		}

		if (isobus::SystemTiming::time_expired_ms(rate.lastRcMs, RATE_TIMEOUT_MS))
		{
			// FALLBACK (step 3 D1): nothing from RC (no setpoint, no 158 = 0). Per element on RC's rate: its default
			// rate with 287 = 2 if the DDOP has one, else hand back (158 = 0); without either, 0 (stop). OFF and
			// REFUSED stay as they are.
			log("Rate") << "Channel " << int(channel) << ": nothing from RateController for " << RATE_TIMEOUT_MS / 1000 << " s, fallback" << std::endl;
			rate.lost = true;
			bool alerted = false;
			for (auto &client : clients)
			{
				for (auto &element : client.second.get_rate_elements_mutable())
				{
					if (element.channel != channel || (element.rxState != RxState::Starting && element.rxState != RxState::Active))
					{
						continue;
					}

					std::string how;
					if (element.hasDefault)
					{
						constexpr std::uint16_t DDI_RX_SOURCE = 287;
						element.rxState = RxState::Fallback;
						element.rxConfirming = false;
						if (element.has287)
						{
							send_set_value(client.first, DDI_RX_SOURCE, element.sourceElement, 2); // 2 = default rate
						}
						send_set_value(client.first, element.ddi, element.elementNumber, element.defaultRate);
						how = "default rate " + std::to_string(element.defaultRate);
					}
					else if (element.has158 && element.rxReported != 3)
					{
						hand_back_rate_element(client.first, element, RxState::Fallback);
						how = "handed back (158 = 0)";
					}
					else
					{
						element.rxState = RxState::Fallback;
						send_set_value(client.first, element.ddi, element.elementNumber, 0);
						how = "stopped (0)";
					}
					log("Rate") << rate_element_text(element) << ": fallback, " << how << std::endl;
					pendingRateAlerts.push_back(element.designator + ": RC lost, " + how);
					alerted = true;
				}
			}
			if (!alerted)
			{
				// no element on RC's rate (VR off, or OFF / REFUSED): nothing changes on the implement, but RC is gone
				pendingRateAlerts.push_back("Channel " + std::to_string(int(channel)) + ": RC lost (implements on their own rate)");
			}
		}
		else if (rate.vrOn && rate.lastReceivedMs != 0 && isobus::SystemTiming::time_expired_ms(rate.lastSentMs, RATE_RESEND_MS))
		{
			// sent continuously to the elements on RC's rate, so one that reconnects gets it within a second
			send_rate_to_elements(channel, rate.ddi, rate.value);
			rate.lastSentMs = now;
		}
	}
}

std::uint16_t MyTCServer::rate_channel_ddi(std::uint8_t channel) const
{
	const RateChannel &rate = rateChannels[channel - 1];
	return rate.infoReceived ? rate.infoDdi : rate.ddi;
}

void MyTCServer::update_rate_channel_info(std::uint8_t channel, std::uint16_t ddi, const std::string &name)
{
	if (channel < 1 || channel > MAX_RATE_CHANNELS)
	{
		return;
	}
	if (!is_rate_setpoint_ddi(ddi))
	{
		ddi = 0; // not a rate type: the channel is not in use
	}

	std::lock_guard<std::recursive_mutex> lock(clientsMutex);
	RateChannel &rate = rateChannels[channel - 1];
	const std::uint16_t before = rate_channel_ddi(channel);
	const bool first = !rate.infoReceived;
	const bool renamed = rate.name != name;
	rate.infoReceived = true;
	rate.infoDdi = ddi;
	rate.name = name;

	if (first || renamed || before != ddi)
	{
		log("Rate") << "Channel " << int(channel) << " \"" << name << "\": " << (ddi != 0 ? "DDI " + std::to_string(ddi) : std::string("not in use")) << std::endl;
	}
	if (before != ddi)
	{
		match_rate_elements();
	}
	else if (ddi != 0 && (first || renamed))
	{
		// same type, new name (or the first info after the setpoints already matched elements): log their mapping with it
		for (auto &client : clients)
		{
			for (const auto &element : client.second.get_rate_elements())
			{
				if (element.channel == channel)
				{
					log("Rate") << "Element " << element.elementNumber << " \"" << element.designator << "\" (DDI " << element.ddi
					            << ") -> channel " << int(channel) << " \"" << name << "\"" << std::endl;
					pendingRateMessages.push_back(element.designator + " -> " + (name.empty() ? "channel " + std::to_string(channel) : name));
				}
			}
		}
	}
}

std::vector<std::string> MyTCServer::take_rate_messages()
{
	std::lock_guard<std::recursive_mutex> lock(clientsMutex);
	std::vector<std::string> result;
	result.swap(pendingRateMessages);
	return result;
}

void MyTCServer::subscribe_rate_actuals(std::shared_ptr<isobus::ControlFunction> client, ClientState &state, RateElement &element)
{
	using Trigger = isobus::task_controller_object::DeviceProcessDataObject::AvailableTriggerMethods;
	auto &pool = state.get_pool();

	// time interval 1 s if offered, else on change, else the caller requests it every second
	auto subscribe = [&](isobus::task_controller_object::DeviceProcessDataObject &processData, std::uint16_t elementNumber, bool &poll) -> std::string {
		std::string how;
		if (processData.has_trigger_method(Trigger::TimeInterval))
		{
			send_time_interval_measurement_command(client, processData.get_ddi(), elementNumber, 1000);
			how = "every 1 s";
		}
		else if (processData.has_trigger_method(Trigger::OnChange))
		{
			send_change_threshold_measurement_command(client, processData.get_ddi(), elementNumber, 1);
			how = "on change";
		}
		else
		{
			poll = true;
			how = "requested every 1 s";
		}
		return how;
	};

	std::string actualText = "no actual rate";
	auto rateElement = element_by_number(pool, element.elementNumber);
	if (rateElement)
	{
		auto actual = child_process_data(pool, *rateElement, element.ddi + 1); // 1 -> 2, 6 -> 7, 11 -> 12
		if (actual)
		{
			element.actualDdi = actual->get_ddi();
			actualText = "actual DDI " + std::to_string(element.actualDdi) + " " + subscribe(*actual, element.elementNumber, element.pollActual);
		}
	}

	// a total of the same kind, the finer one first, on the element or the nearest parent that has one
	std::vector<std::uint16_t> totalDdis;
	if (element.ddi == static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SetpointVolumePerAreaApplicationRate))
	{
		totalDdis = { static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ApplicationTotalVolume_ml), static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ApplicationTotalVolume_L) };
	}
	else if (element.ddi == static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SetpointMassPerAreaApplicationRate))
	{
		totalDdis = { static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ApplicationTotalMassGram_g), static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ApplicationTotalMass_kg) };
	}
	else
	{
		totalDdis = { static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ApplicationTotalCount) };
	}

	std::string totalText = "no total";
	auto level = rateElement;
	for (int depth = 0; level && element.totalDdi == 0 && depth < 8; depth++)
	{
		for (std::uint16_t ddi : totalDdis)
		{
			auto total = (element.totalDdi == 0) ? child_process_data(pool, *level, ddi) : nullptr;
			if (total)
			{
				element.totalDdi = ddi;
				element.totalElement = level->get_element_number();
				totalText = "total DDI " + std::to_string(ddi) + " on element " + std::to_string(element.totalElement) + " " + subscribe(*total, element.totalElement, element.pollTotal);
			}
		}

		// up to the parent element (the device's parent is not an element)
		auto parent = pool.get_object_by_id(level->get_parent_object());
		level = (parent && parent->get_object_type() == isobus::task_controller_object::ObjectTypes::DeviceElement && parent != level) ?
		  std::dynamic_pointer_cast<isobus::task_controller_object::DeviceElementObject>(parent) :
		  nullptr;
	}

	log("Rate") << "Element " << element.elementNumber << " \"" << element.designator << "\": " << actualText << ", " << totalText << std::endl;
}

std::vector<RateChannelActuals> MyTCServer::get_rate_channel_actuals()
{
	std::lock_guard<std::recursive_mutex> lock(clientsMutex);
	const std::uint32_t now = isobus::SystemTiming::get_timestamp_ms();
	std::vector<RateChannelActuals> result;

	for (std::uint8_t channel = 1; channel <= MAX_RATE_CHANNELS; channel++)
	{
		RateChannelActuals actuals;
		actuals.channel = channel;
		std::int64_t rateSum = 0;
		std::uint8_t rateCount = 0;
		std::int64_t totalSum = 0;
		bool totalSeen = false;
		std::vector<std::pair<isobus::ControlFunction *, std::uint32_t>> totalsCounted; // a parent's total once
		bool any158 = false;
		bool all158On = true;
		std::int32_t lowestOther = 3;
		bool noReply = false;
		bool modeSeen = false;

		for (auto &client : clients)
		{
			for (auto &element : client.second.get_rate_elements_mutable())
			{
				if (element.channel != channel)
				{
					continue;
				}
				actuals.elements++;

				// 158 = 1 only if every element with 158 reports 1, else the lowest other value; 288 from the first
				if (element.has158)
				{
					any158 = true;
					const std::int32_t reported = (element.rxReported < 0) ? 0 : element.rxReported;
					if (reported != 1)
					{
						all158On = false;
						lowestOther = std::min(lowestOther, reported);
					}
				}
				noReply = noReply || element.noReply;
				if (element.has288 && !modeSeen)
				{
					modeSeen = true;
					actuals.mode = static_cast<std::uint8_t>(std::max<std::int32_t>(0, std::min<std::int32_t>(element.modeReported, 255)));
				}

				// values the implement only sends on request
				if (element.pollActual)
				{
					send_request_value(client.first, element.actualDdi, element.elementNumber);
				}
				if (element.pollTotal)
				{
					send_request_value(client.first, element.totalDdi, element.totalElement);
				}

				if (element.actualDdi != 0 && element.actualMs != 0 && !isobus::SystemTiming::time_expired_ms(element.actualMs, RATE_ACTUAL_FRESH_MS))
				{
					actuals.actualDdi = element.actualDdi;
					rateSum += element.actualValue;
					rateCount++;
				}

				const std::pair<isobus::ControlFunction *, std::uint32_t> totalKey(client.first.get(), (static_cast<std::uint32_t>(element.totalElement) << 16) | element.totalDdi);
				if (element.totalDdi != 0 && element.totalMs != 0 && !isobus::SystemTiming::time_expired_ms(element.totalMs, RATE_ACTUAL_FRESH_MS) &&
				    std::find(totalsCounted.begin(), totalsCounted.end(), totalKey) == totalsCounted.end())
				{
					totalsCounted.push_back(totalKey);
					// in mL, g or count: DDI 80 (L) and 81 (kg) x 1000
					std::int64_t value = element.totalValue;
					std::uint16_t fineDdi = element.totalDdi;
					if (element.totalDdi == static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ApplicationTotalVolume_L))
					{
						value *= 1000;
						fineDdi = static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ApplicationTotalVolume_ml);
					}
					else if (element.totalDdi == static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ApplicationTotalMass_kg))
					{
						value *= 1000;
						fineDdi = static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ApplicationTotalMassGram_g);
					}
					actuals.totalDdi = fineDdi;
					totalSum += value;
					totalSeen = true;
				}
			}
		}

		if (rateCount > 0)
		{
			actuals.rate = static_cast<std::int32_t>(rateSum / rateCount);
			actuals.flags |= 0x01;
		}
		if (totalSeen)
		{
			actuals.total = static_cast<std::int32_t>(std::min<std::int64_t>(totalSum, INT32_MAX));
			actuals.flags |= 0x02;
		}
		if (any158)
		{
			const std::int32_t state = all158On ? 1 : lowestOther;
			actuals.flags |= static_cast<std::uint8_t>((state & 0x03) << 2);
			actuals.flags |= 0x10;
		}
		if (noReply)
		{
			actuals.flags |= 0x20;
		}

		// every channel RC announced (0xF4 DDI != 0), with elements = 0 when none are matched, so RC can tell
		// "TC there, nothing matched" from "no TC"; and once more after a channel loses its elements
		const bool hasElements = actuals.elements > 0;
		const bool announced = rateChannels[channel - 1].infoReceived && rateChannels[channel - 1].infoDdi != 0;
		if (hasElements || announced || actualsReported[channel - 1])
		{
			result.push_back(actuals);
		}
		if (hasElements != actualsReported[channel - 1])
		{
			log("Rate") << "Channel " << int(channel) << ": " << (hasElements ? "reporting actuals to RateController" : "no elements, actuals stopped") << std::endl;
		}
		actualsReported[channel - 1] = hasElements;
	}
	return result;
}

void MyTCServer::match_rate_elements()
{
	for (auto &client : clients)
	{
		match_rate_elements(client.first, client.second, false);
	}
}

void MyTCServer::match_rate_elements(std::shared_ptr<isobus::ControlFunction> client, ClientState &state, bool reportAll)
{
	const std::vector<RateElement> elements = state.get_rate_elements(); // a copy: channels are set below
	for (std::size_t i = 0; i < elements.size(); i++)
	{
		const RateElement &element = elements[i];

		// the channels of the element's type; exactly one is a match, two or more need an assignment
		std::uint8_t match = 0;
		std::uint8_t count = 0;
		std::string sameType;
		for (std::uint8_t channel = 1; channel <= MAX_RATE_CHANNELS; channel++)
		{
			if (rate_channel_ddi(channel) == element.ddi)
			{
				match = channel;
				count++;
				sameType += (sameType.empty() ? "" : ", ") + std::to_string(channel) + " \"" + rateChannels[channel - 1].name + "\"";
			}
		}
		const std::uint8_t channel = (count == 1) ? match : 0;
		const bool changed = (channel != element.channel);

		if (changed)
		{
			RateElement &live = state.get_rate_elements_mutable()[i];
			if (element.channel != 0 && live.rxState != RxState::Off)
			{
				hand_back_rate_element(client, live, RxState::Off); // leaves its channel: back to its own rate (no 158: 0)
			}
			state.set_rate_element_channel(i, channel);

			const RateChannel &rate = rateChannels[channel == 0 ? 0 : channel - 1];
			if (channel != 0 && rate.vrOn && rate.lastReceivedMs != 0 && !rate.lost && rate.ddi == element.ddi)
			{
				start_rate_element(client, live, rate); // the channel's rate at once, as a fresh request
			}
		}

		if (changed || reportAll)
		{
			const std::string elementText = "Element " + std::to_string(element.elementNumber) + " \"" + element.designator + "\" (DDI " + std::to_string(element.ddi) + ")";
			std::string logText;
			std::string messageText;
			if (channel != 0)
			{
				const std::string &name = rateChannels[channel - 1].name;
				logText = elementText + " -> channel " + std::to_string(channel) + " \"" + name + "\"";
				messageText = element.designator + " -> " + (name.empty() ? "channel " + std::to_string(channel) : name);
			}
			else if (count > 1)
			{
				logText = elementText + ": channels " + sameType + " are the same type, assignment needed";
				messageText = element.designator + ": assignment needed";
			}
			else
			{
				logText = elementText + ": no RateController product of this type";
				messageText = element.designator + ": no RC product of this type";
			}
			log("Rate") << logText << std::endl;
			pendingRateMessages.push_back(messageText);
		}
	}
}

void MyTCServer::send_gnss_quality(std::uint8_t quality)
{
	std::lock_guard<std::recursive_mutex> lock(clientsMutex);
	constexpr auto DDI = isobus::DataDescriptionIndex::GNSSQuality;
	for (auto &client : clients)
	{
		if (client.second.has_element_number_for_ddi(DDI))
		{
			send_set_value(client.first, static_cast<std::uint16_t>(DDI), client.second.get_element_number_for_ddi(DDI), static_cast<std::int32_t>(quality));
		}
	}
}

void MyTCServer::send_section_setpoint_states(std::shared_ptr<isobus::ControlFunction> client, std::uint8_t ddiOffset, bool forceWorkState)
{
	std::lock_guard<std::recursive_mutex> lock(clientsMutex);
	std::uint8_t sectionOffset = ddiOffset * NUMBER_SECTIONS_PER_CONDENSED_MESSAGE;
	std::uint32_t value = 0;
	for (std::uint8_t i = 0; i < NUMBER_SECTIONS_PER_CONDENSED_MESSAGE; i++)
	{
		value |= (clients[client].get_section_setpoint_state(sectionOffset + i) << (2 * i));
	}

	// Modern ECU? (DDI 290  SetpointCondensedWorkState1_16 exists)
	std::uint16_t ddiTarget = static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SetpointCondensedWorkState1_16) + ddiOffset;
	// Legacy ECU? (DDI 161  ActualCondensedWorkState1_16 exists and Settable)
	std::uint16_t ddiTargetLegacy = static_cast<std::uint16_t>(isobus::DataDescriptionIndex::ActualCondensedWorkState1_16) + ddiOffset;
	if (clients[client].has_element_number_for_ddi(static_cast<isobus::DataDescriptionIndex>(ddiTarget)))
	{
		std::uint16_t elementNumber = clients[client].get_element_number_for_ddi(static_cast<isobus::DataDescriptionIndex>(ddiTarget));
		send_set_value(client, ddiTarget, elementNumber, value);

		bool setpointWorkState = clients[client].is_any_section_setpoint_on();
		if ((forceWorkState || (clients[client].get_setpoint_work_state() != setpointWorkState)) && clients[client].has_element_number_for_ddi(isobus::DataDescriptionIndex::SetpointWorkState))
		{
			send_set_value(client, static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SetpointWorkState), clients[client].get_element_number_for_ddi(isobus::DataDescriptionIndex::SetpointWorkState), setpointWorkState ? 1 : 0);
			clients[client].set_setpoint_work_state(setpointWorkState);
		}
		else if (!forceWorkState && !clients[client].has_element_number_for_ddi(isobus::DataDescriptionIndex::SetpointWorkState))
		{
			log("TC Server") << "DDI 289 (SetpointWorkState) not available!" << std::endl;
		}
		return; // Modern condensed path complete
	}
	else if (clients[client].has_element_number_for_ddi(static_cast<isobus::DataDescriptionIndex>(ddiTargetLegacy)))
	{
		if (is_ddi_settable(client, ddiTargetLegacy))
		{
			send_set_value(client, ddiTargetLegacy, clients[client].get_element_number_for_ddi(static_cast<isobus::DataDescriptionIndex>(ddiTargetLegacy)), value);
		}
		else
		{
			log("TC Server") << "Legacy DDI " << ddiTargetLegacy << " (ActualCondensedWorkState) is not settable!" << std::endl;
		}
		return; // Legacy condensed path complete
	}
	else if (clients[client].uses_per_element_control())
	{
		// Per-element control: send setpoints to each section element individually (DDI 141 settable)
		std::uint16_t setpointDDI = clients[client].get_per_element_setpoint_ddi();
		for (std::uint8_t i = 0; i < NUMBER_SECTIONS_PER_CONDENSED_MESSAGE; i++)
		{
			std::uint8_t sectionIndex = sectionOffset + i;
			if (sectionIndex >= clients[client].get_number_of_sections())
			{
				break;
			}
			std::uint16_t elementNumber = clients[client].get_element_number_for_section(sectionIndex);
			if (elementNumber != 0)
			{
				std::uint8_t state = clients[client].get_section_setpoint_state(sectionIndex);
				send_set_value(client, setpointDDI, elementNumber, (state == SectionState::ON) ? 1 : 0);
			}
		}

		// Also send global work state on the boom/device element if DDI 289 is available
		bool setpointWorkState = clients[client].is_any_section_setpoint_on();
		if (forceWorkState || (clients[client].get_setpoint_work_state() != setpointWorkState))
		{
			if (clients[client].has_element_number_for_ddi(isobus::DataDescriptionIndex::SetpointWorkState))
			{
				send_set_value(client,
				               static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SetpointWorkState),
				               clients[client].get_element_number_for_ddi(isobus::DataDescriptionIndex::SetpointWorkState),
				               setpointWorkState ? 1 : 0);
				clients[client].set_setpoint_work_state(setpointWorkState);
			}
		}
		return; // Per-element path complete
	}
	else
	{
		log("TC Server") << "No supported method to send section setpoint states! "
		                 << "Device has no DDI 290, 161 (settable), or 141 (settable)." << std::endl;
	}
}

void MyTCServer::send_section_control_state(std::shared_ptr<isobus::ControlFunction> client, bool enabled)
{
	std::lock_guard<std::recursive_mutex> lock(clientsMutex);
	send_set_value(client, static_cast<std::uint16_t>(isobus::DataDescriptionIndex::SectionControlState), clients[client].get_element_number_for_ddi(isobus::DataDescriptionIndex::SectionControlState), enabled ? 1 : 0);
}

bool MyTCServer::is_ddi_settable(std::shared_ptr<isobus::ControlFunction> client, std::uint16_t ddi)
{
	std::lock_guard<std::recursive_mutex> lock(clientsMutex);
	for (std::uint32_t i = 0; i < clients[client].get_pool().size(); i++)
	{
		auto object = clients[client].get_pool().get_object_by_index(i);
		if (object && object->get_object_type() == isobus::task_controller_object::ObjectTypes::DeviceProcessData)
		{
			auto processDataObject = std::dynamic_pointer_cast<isobus::task_controller_object::DeviceProcessDataObject>(object);
			if (processDataObject->get_ddi() == ddi)
			{
				return processDataObject->has_property(isobus::task_controller_object::DeviceProcessDataObject::PropertiesBit::Settable);
			}
		}
	}
	return false;
}
