/**
 * @author Daan Steenbergen
 * @brief An ISOBUS Task Controller for AgOpenGPS
 * @version 0.1
 * @date 2025-1-20
 *
 * @copyright 2025 Daan Steenbergen
 */

#pragma once

#include "element_hierarchy.hpp"
#include "isobus/isobus/isobus_data_dictionary.hpp"
#include "isobus/isobus/isobus_device_descriptor_object_pool.hpp"
#include "isobus/isobus/isobus_standard_data_description_indices.hpp"
#include "isobus/isobus/isobus_task_controller_server.hpp"

#include <array>
#include <cstdint>
#include <map>
#include <mutex>
#include <optional>
#include <queue>
#include <string>
#include <vector>

constexpr std::uint8_t NUMBER_SECTIONS_PER_CONDENSED_MESSAGE = 16;

enum SectionState : std::uint8_t
{
	OFF = 0, ///< Section is off
	ON = 1, ///< Section is on
	ERROR_SATE = 2, ///< Section is in an error state
	NOT_INSTALLED = 3 ///< Section is not installed
};

/// @brief A rate element's prescription state (Group2_Step3_Prescription_State_Design.md § 5)
enum class RxState : std::uint8_t
{
	Off = 0, ///< No RC rate: VR off or the channel not in use; the implement uses its own rate
	Starting = 1, ///< 287 = 1, 158 = 1 and the setpoint sent; waiting for the implement to report 158 = 1
	Active = 2, ///< The implement takes RC's rate (158 = 1, or the element has no 158)
	Refused = 3, ///< The implement turned 158 off, refused it, reports an error, or never replied; only "Ask again" restarts it
	Fallback = 4 ///< Nothing from RC for RATE_TIMEOUT_MS: its default rate (287 = 2), else handed back (158 = 0)
};

/// @brief An implement element that takes a rate setpoint, and the rate channel that drives it
struct RateElement
{
	std::uint16_t ddi; ///< Settable rate DDI: 1 volume, 6 mass or 11 count per area
	std::uint16_t elementNumber; ///< The element the DDI belongs to
	std::uint8_t channel; ///< Rate channel (1 = RateController product 1), 0 = not matched to a channel
	std::string designator; ///< The element's DDOP designator, for the log
	std::uint16_t actualDdi = 0; ///< Its actual rate DDI (2, 7 or 12) when the DDOP declares it, else 0
	std::uint16_t totalDdi = 0; ///< An application total of the same kind (80/81/82/351/352) on it or a parent, else 0
	std::uint16_t totalElement = 0; ///< The element the total belongs to (may be a parent, shared by several rate elements)
	std::int32_t actualValue = 0; ///< Last actual rate from the implement, raw
	std::uint32_t actualMs = 0; ///< When it arrived, 0 = never
	std::int32_t totalValue = 0; ///< Last total from the implement, raw
	std::uint32_t totalMs = 0; ///< When it arrived, 0 = never
	bool pollActual = false; ///< The implement offers no time-interval or on-change trigger: request it every second
	bool pollTotal = false; ///< As pollActual, for the total

	// prescription control (step 3): each DDI on the element or its nearest parent that has it
	bool has158 = false; ///< DDI 158 Prescription Control State is declared
	std::uint16_t rxElement = 0; ///< The element that has 158
	bool has287 = false; ///< DDI 287 Setpoint Prescription Mode is declared and settable
	std::uint16_t sourceElement = 0; ///< The element that has 287
	bool has288 = false; ///< DDI 288 Actual Prescription Mode is declared
	std::uint16_t modeElement = 0; ///< The element that has 288
	bool hasDefault = false; ///< The default rate (DDI 3 / 8 / 13) is known
	std::int32_t defaultRate = 0; ///< In the rate DDI's unit
	std::uint16_t defaultElement = 0; ///< The element of a default rate held as process data (requested), 0 = none or a property

	RxState rxState = RxState::Off;
	std::int32_t rxReported = -1; ///< 158 as the implement last reported it, -1 = never
	std::uint32_t rxReportedMs = 0; ///< When, 0 = never
	std::int32_t modeReported = 0; ///< 288 as last reported, 0 = unknown
	std::uint8_t rxWanted = 0; ///< The 158 value last commanded (1 on, 0 off)
	bool rxConfirming = false; ///< Waiting for the implement to confirm rxWanted
	std::uint32_t rxCommandMs = 0; ///< When rxWanted was last commanded
	bool rxAsked = false; ///< 158 requested since the last command
	std::uint32_t rxAskedMs = 0; ///< When it was requested
	std::uint8_t rxTries = 0; ///< Commands sent for this request (RX_TRIES without a reply = no reply)
	bool noReply = false; ///< No reply to 158 after the resends; cleared when it reports 158 or a new request starts
};

/// @brief What one rate channel's elements report, for RateController (PGN 0xF5)
struct RateChannelActuals
{
	std::uint8_t channel = 0; ///< 1 to 5
	std::uint16_t actualDdi = 0; ///< 2, 7 or 12
	std::int32_t rate = 0; ///< Mean of the elements' actual rates, raw in actualDdi's unit
	std::uint16_t totalDdi = 0; ///< 351 (mL), 352 (g) or 82 (count); 0 = no total
	std::int32_t total = 0; ///< Sum of the elements' totals (a parent's total counted once), in totalDdi's unit
	std::uint8_t elements = 0; ///< Elements matched to the channel
	std::uint8_t flags = 0; ///< bit 0 rate valid, bit 1 total valid, bits 2-3 158 as reported, bit 4 has 158, bit 5 no reply
	std::uint8_t mode = 0; ///< 288 as the first element with it reports it, 0 = unknown / not present
};

class ClientState
{
public:
	void set_number_of_sections(std::uint8_t number);
	void set_section_setpoint_state(std::uint8_t section, std::uint8_t state);
	void set_section_actual_state(std::uint8_t section, std::uint8_t state);
	std::uint8_t get_number_of_sections() const;
	std::uint8_t get_section_setpoint_state(std::uint8_t section) const;
	std::uint8_t get_section_actual_state(std::uint8_t section) const;
	std::uint16_t get_element_number_for_section(std::uint8_t section) const;
	void set_element_number_for_section(std::uint8_t section, std::uint16_t elementNumber);
	bool try_get_section_for_element(std::uint16_t elementNumber, std::uint8_t &section) const;
	bool is_any_section_setpoint_on() const;
	bool get_setpoint_work_state() const;
	void set_setpoint_work_state(bool state);
	bool get_actual_work_state() const;
	void set_actual_work_state(bool state);
	bool is_section_control_enabled() const;
	void set_section_control_enabled(bool state);
	bool uses_per_element_control() const;
	void set_uses_per_element_control(bool state);
	std::uint16_t get_per_element_setpoint_ddi() const;
	void set_per_element_setpoint_ddi(std::uint16_t ddi);
	isobus::DeviceDescriptorObjectPool &get_pool();
	bool are_measurement_commands_sent() const;
	void mark_measurement_commands_sent();
	/// @brief Schedules the TC's section control state to be applied to this client at the given time.
	void schedule_initial_section_control(std::uint32_t dueMs);
	/// @brief Returns true (once) when the scheduled initial section control state is due.
	bool take_due_initial_section_control(std::uint32_t nowMs);
	std::uint16_t get_element_number_for_ddi(isobus::DataDescriptionIndex ddi) const;
	void set_element_number_for_ddi(isobus::DataDescriptionIndex ddi, std::uint16_t elementNumber);
	bool has_element_number_for_ddi(isobus::DataDescriptionIndex ddi) const;
	bool is_element_or_parent_off(std::uint16_t elementNumber) const; ///< Checks if the element or any of its parents is off
	/// @brief Caches each element's parent from the pool. Call once the pool is deserialized.
	void build_element_hierarchy();
	// Element work state management these act like master / override for actual sections
	void set_element_work_state(std::uint16_t elementNumber, bool isWorking);
	bool try_get_element_work_state(std::uint16_t elementNumber, bool &isWorking) const;
	void clear_rate_elements();
	void add_rate_element(const RateElement &element);
	const std::vector<RateElement> &get_rate_elements() const;
	std::vector<RateElement> &get_rate_elements_mutable();
	void set_rate_element_channel(std::size_t index, std::uint8_t channel);
	/// @brief Stores an actual rate or total the implement reported for one of the rate elements
	void store_rate_value(std::uint16_t elementNumber, std::uint16_t ddi, std::int32_t value, std::uint32_t nowMs);

private:
	isobus::DeviceDescriptorObjectPool pool; ///< The device descriptor object pool (DDOP) for the TC
	bool areMeasurementCommandsSent = false; ///< Whether or not the measurement commands have been sent
	std::optional<std::uint32_t> initialSectionControlDueMs; ///< When to apply the TC's section control state after connect
	std::map<isobus::DataDescriptionIndex, std::uint16_t> ddiToElementNumber; ///< Mapping of DDI to element number // TODO: better way to do this?

	std::uint8_t numberOfSections;
	std::vector<std::uint8_t> sectionSetpointStates; // 2 bits per section (0 = off, 1 = on, 2 = error, 3 = not installed)
	std::vector<std::uint8_t> sectionActualStates; // 2 bits per section (0 = off, 1 = on, 2 = error, 3 = not installed)
	std::vector<std::uint16_t> sectionToElementNumber; // Maps section index to element number for hierarchy checking
	std::map<std::uint16_t, std::uint8_t> elementToSection; ///< Reverse mapping: element number -> section index
	bool setpointWorkState = false; ///< The overall work state desired (DDI 289)
	bool actualWorkState = false; ///< The overall work state actual
	std::map<std::uint16_t, bool> elementWorkStates; ///< Work state per element (element number -> is working)
	element_hierarchy::ParentMap elementParents; ///< Parent of each element, built once at pool activation
	bool isSectionControlEnabled = false; ///< Stores auto vs manual mode setting
	bool usesPerElementControl = false; ///< Legacy mode: use per-element setpoint instead of condensed
	std::uint16_t perElementSetpointDDI = 0; ///< The DDI to use for per-element setpoints (289 or 141), 0 if not applicable
	std::vector<RateElement> rateElements; ///< Elements with a settable rate DDI (an element may appear more than once)
};

// Create the task controller server object, this will handle all the ISOBUS communication for us
class MyTCServer : public isobus::TaskControllerServer
{
public:
	/// @param tcGeo Announce TC-GEO without position-based control (version reply option bit 1): the TC sends rate setpoints
	MyTCServer(std::shared_ptr<isobus::InternalControlFunction> internalControlFunction,
	           isobus::TaskControllerServer::TaskControllerVersion version = isobus::TaskControllerServer::TaskControllerVersion::SecondEditionDraft,
	           bool tcGeo = false);
	bool activate_object_pool(std::shared_ptr<isobus::ControlFunction> partnerCF, ObjectPoolActivationError &, ObjectPoolErrorCodes &, std::uint16_t &, std::uint16_t &) override;
	bool change_designator(std::shared_ptr<isobus::ControlFunction>, std::uint16_t, const std::vector<std::uint8_t> &) override;
	bool deactivate_object_pool(std::shared_ptr<isobus::ControlFunction> partnerCF) override;
	bool delete_device_descriptor_object_pool(std::shared_ptr<isobus::ControlFunction> partnerCF, ObjectPoolDeletionErrors &) override;
	bool get_is_stored_device_descriptor_object_pool_by_structure_label(std::shared_ptr<isobus::ControlFunction>, const std::vector<std::uint8_t> &, const std::vector<std::uint8_t> &) override;
	bool get_is_stored_device_descriptor_object_pool_by_localization_label(std::shared_ptr<isobus::ControlFunction>, const std::array<std::uint8_t, 7> &) override;
	bool get_is_enough_memory_available(std::uint32_t) override;
	void identify_task_controller(std::uint8_t) override;
	void on_client_timeout(std::shared_ptr<isobus::ControlFunction> partner) override;
	void on_client_version_received(std::shared_ptr<isobus::ControlFunction> clientControlFunction, std::uint8_t version) override;
	void on_process_data_acknowledge(std::shared_ptr<isobus::ControlFunction> partner, std::uint16_t dataDescriptionIndex, std::uint16_t elementNumber, std::uint8_t errorCodesFromClient, ProcessDataCommands processDataCommand) override;
	bool on_value_command(std::shared_ptr<isobus::ControlFunction> partner,
	                      std::uint16_t dataDescriptionIndex,
	                      std::uint16_t elementNumber,
	                      std::int32_t processDataValue,
	                      std::uint8_t &errorCodes) override;
	bool store_device_descriptor_object_pool(std::shared_ptr<isobus::ControlFunction> partnerCF, const std::vector<std::uint8_t> &binaryPool, bool appendToPool) override;

	/// @brief Returns a snapshot copy of the client map, not a live reference.
	/// See the concurrency note on clientsMutex below for why: the isobus stack
	/// invokes the TaskControllerServer overrides above from its own background
	/// thread, concurrently with whichever thread calls this. A returned reference
	/// could be mutated (even reallocated, on insert/erase) out from under a caller
	/// mid-iteration — a returned copy can't.
	std::map<std::shared_ptr<isobus::ControlFunction>, ClientState> get_clients();
	void request_measurement_commands();
	void update_section_states(std::vector<bool> &sectionStates);
	void update_section_control_enabled(bool enabled);

	/// @brief Sets whether implements are switched to section control when they connect (settings: sectionControlOnConnect).
	/// @details When on, section control starts enabled for implements that connect before AOG asks for a state.
	/// When off, nothing is sent on connect, as before; AOG's SC button still switches every connected client.
	void set_section_control_on_connect(bool enabled);

	/// @brief Sends GNSS quality (DDI 514) to every client that declares that DDI.
	/// @param quality NMEA 2000 GNSS Method: 0=No GNSS, 1=GNSS, 2=DGNSS, 3=Precise, 4=RTK Fixed, 5=RTK Float, 6=Estimated, 7=Manual, 8=Simulated
	void send_gnss_quality(std::uint8_t quality);

	static constexpr std::uint8_t MAX_RATE_CHANNELS = 5; ///< RateController's rate products

	/// @brief True for the rate setpoint DDIs a rate channel can carry: 1 (volume), 6 (mass), 11 (count) per area
	static bool is_rate_setpoint_ddi(std::uint16_t ddi);

	/// @brief Stores a rate setpoint from RateController (PGN 0xF2) and sends it to the channel's elements when it changed
	/// @param channel Rate channel 1 to MAX_RATE_CHANNELS (RateController's product + 1); others are ignored
	/// @param ddi The rate setpoint DDI (see is_rate_setpoint_ddi), in that DDI's ISO unit
	/// @param value The setpoint; negative values are sent as 0
	void update_rate_setpoint(std::uint8_t channel, std::uint16_t ddi, std::int32_t value);

	/// @brief Call every update: resends each channel's setpoint every RATE_RESEND_MS, and sends 0 once
	/// when a channel has had no setpoint for RATE_TIMEOUT_MS (lose the rate source, stop applying)
	void update_rate_setpoints();

	/// @brief RateController's prescription request for a channel (PGN 0xF2 with DDI 158): 0 = VR off, hand the channel's
	/// elements back to their own rate; 1 = "Ask again", a fresh request for elements that refused or didn't reply
	void update_rate_rx_request(std::uint8_t channel, std::int32_t value);

	/// @brief Returns (and clears) rate alarms since the last call (refused, no reply, fallback), for AOG's hardware message
	std::vector<std::string> take_rate_alerts();

	/// @brief Stores a channel's type and name from RateController (PGN 0xF4) and re-matches the rate elements when the type changed
	/// @param channel Rate channel 1 to MAX_RATE_CHANNELS; others are ignored
	/// @param ddi The channel's rate DDI (see is_rate_setpoint_ddi), 0 = channel not in use
	/// @param name RateController's product name, for the log
	void update_rate_channel_info(std::uint8_t channel, std::uint16_t ddi, const std::string &name);

	/// @brief Returns (and clears) the element-to-channel changes since the last call, for AOG's hardware message
	std::vector<std::string> take_rate_messages();

	/// @brief Each channel's actual rate and total from its elements, for PGN 0xF5. Includes, once, a channel
	/// that just lost its elements (elements 0, flags 0). Call about once a second; also requests totals that
	/// the implement only sends on request.
	std::vector<RateChannelActuals> get_rate_channel_actuals();

private:
	struct RateChannel
	{
		std::uint16_t ddi = 0; ///< DDI of the last setpoint, 0 = never received
		std::int32_t value = 0; ///< Last setpoint
		std::uint32_t lastReceivedMs = 0; ///< When the last setpoint arrived
		std::uint32_t lastSentMs = 0; ///< When it was last sent to the elements
		bool lost = false; ///< Nothing from RC for RATE_TIMEOUT_MS: elements in fallback; waits for RC
		bool vrOn = false; ///< RC sends setpoints (VR on); false after its DDI 158 = 0 (VR off)
		bool vrOffLogged = false; ///< "VR off" logged once: RC started with VR off sends no setpoint, only 158 = 0 every second
		std::uint32_t lastRcMs = 0; ///< When RC last sent anything for the channel (setpoint or DDI 158), 0 = never
		bool infoReceived = false; ///< Channel info (PGN 0xF4) has arrived at least once
		std::uint16_t infoDdi = 0; ///< The channel's type from its info, 0 = not in use
		std::string name; ///< RateController's product name from the channel info
	};

	/// @brief The channel's type for matching: from its info once that has arrived, else from its setpoints (older senders)
	std::uint16_t rate_channel_ddi(std::uint8_t channel) const;

	/// @brief Matches every rate element of every client to the channel of the same type, sends 0 to elements that
	/// lose their channel and logs each change. Two or more channels of one type match nothing. Caller must hold clientsMutex.
	void match_rate_elements();
	/// @param reportAll Log every element's result, not only changes (when the implement connects)
	void match_rate_elements(std::shared_ptr<isobus::ControlFunction> client, ClientState &state, bool reportAll);
	static constexpr std::uint32_t RATE_RESEND_MS = 1000;
	static constexpr std::uint32_t RATE_TIMEOUT_MS = 3000; ///< Nothing from RC this long: FALLBACK (step 3 D1)
	static constexpr std::uint32_t RX_CONFIRM_MS = 1000; ///< 158 unconfirmed this long after a command: request it; again: resend
	static constexpr std::uint8_t RX_TRIES = 3; ///< Commands without a reply before "no reply"

	/// @brief Sends the setpoint to every element on the channel that uses the given DDI and takes RC's rate
	/// (STARTING or ACTIVE). Caller must hold clientsMutex.
	void send_rate_to_elements(std::uint8_t channel, std::uint16_t ddi, std::int32_t value);

	/// @brief A fresh request for RC's rate: 287 = 1, 158 = 1, the setpoint, then confirm. An element without 158 is
	/// ACTIVE at once. Caller must hold clientsMutex.
	void start_rate_element(std::shared_ptr<isobus::ControlFunction> client, RateElement &element, const RateChannel &rate);
	/// @brief Hand back to the implement's own rate: 158 = 0 (confirmed), no 0 setpoint; without 158, 0 (stop).
	/// Caller must hold clientsMutex.
	void hand_back_rate_element(std::shared_ptr<isobus::ControlFunction> client, RateElement &element, RxState newState);
	/// @brief Sends element.rxWanted (with 287 and the setpoint for ON) and starts the confirm timer
	void send_rx_command(std::shared_ptr<isobus::ControlFunction> client, RateElement &element, const RateChannel *rate);
	/// @brief Confirms 158 after a command: request after RX_CONFIRM_MS, resend after another, "no reply" after RX_TRIES
	void update_rx_confirm(std::shared_ptr<isobus::ControlFunction> client, RateElement &element, std::uint32_t now);
	/// @brief The implement reported 158 or 288 for an element: state changes, logs and alarms. Caller must hold clientsMutex.
	void on_rx_value(ClientState &state, std::uint16_t ddi, std::uint16_t elementNumber, std::int32_t value);
	/// @brief Finds 158, 287, 288 and the default rate on the element or a parent, and asks for 158 / 288 on change
	void find_rx_ddis(std::shared_ptr<isobus::ControlFunction> client, ClientState &state, RateElement &element);
	static std::string rate_element_text(const RateElement &element);

	/// @brief Sends every condensed setpoint block and DDI 289, even when unchanged (ISO 11783-10 F.3.5.3 b)
	void send_all_section_setpoint_states(std::shared_ptr<isobus::ControlFunction> client);
	void send_section_setpoint_states(std::shared_ptr<isobus::ControlFunction> client, std::uint8_t ddiOffset, bool forceWorkState = false);
	void send_section_control_state(std::shared_ptr<isobus::ControlFunction> client, bool enabled);
	/// @brief Sets the local auto/manual flag and commands DDI 160 if the client supports it. Caller must hold clientsMutex.
	void apply_section_control_state(std::shared_ptr<isobus::ControlFunction> client, ClientState &state, bool enabled);
	bool is_ddi_settable(std::shared_ptr<isobus::ControlFunction> client, std::uint16_t ddi);

	std::map<std::shared_ptr<isobus::ControlFunction>, ClientState> clients;
	std::map<std::shared_ptr<isobus::ControlFunction>, std::queue<std::vector<std::uint8_t>>> uploadedPools;
	bool desiredSectionControlEnabled = false; ///< Applied to implements when they connect; follows AOG's ISOBUS SC button. Guarded by clientsMutex
	bool sectionControlOnConnect = false; ///< Whether desiredSectionControlEnabled is applied on connect at all. Guarded by clientsMutex

	static constexpr std::uint32_t SECTION_DEMAND_TIMEOUT_MS = 2000; ///< AOG sends PGN 229 every frame while a field is open
	std::vector<bool> aogSectionDemand; ///< Last section states from AOG (PGN 229). Guarded by clientsMutex
	std::uint32_t lastSectionDemandMs = 0; ///< When AOG's section demand was last received, 0 = never. Guarded by clientsMutex
	std::array<RateChannel, MAX_RATE_CHANNELS> rateChannels; ///< Rate setpoints from RateController, index = channel - 1. Guarded by clientsMutex
	std::vector<std::string> pendingRateMessages; ///< Element-to-channel changes for AOG's hardware message. Guarded by clientsMutex
	std::vector<std::string> pendingRateAlerts; ///< Refused / no reply / fallback, for AOG's alarm hardware message. Guarded by clientsMutex
	std::array<bool, MAX_RATE_CHANNELS> actualsReported{}; ///< Channel had elements at the last get_rate_channel_actuals. Guarded by clientsMutex
	static constexpr std::uint32_t RATE_ACTUAL_FRESH_MS = 3000; ///< An element's actual or total older than this isn't used

	/// @brief Finds the element's actual rate DDI and a total, and asks the implement to send them. Caller must hold clientsMutex.
	void subscribe_rate_actuals(std::shared_ptr<isobus::ControlFunction> client, ClientState &state, RateElement &element);

	/// @brief Guards clients and uploadedPools.
	///
	/// CONCURRENCY: the isobus/AgIsoStack stack runs its own background thread
	/// (CANHardwareInterface's updateThread) that calls CANNetworkManager::update(),
	/// which is what actually invokes every TaskControllerServer override in this
	/// class (activate_object_pool, on_value_command, ...) — NOT the thread that
	/// runs Application::update(). Meanwhile Application::update() (and the methods
	/// it calls: request_measurement_commands, update_section_states/_control_enabled,
	/// get_clients) reads/writes the SAME maps from the main thread. Without this
	/// lock, that's a concurrent std::map read + insert/erase — undefined behavior,
	/// seen in practice as the TC silently crashing (no exception, no log line)
	/// whenever a new control function appeared on the bus. See docs/CONCURRENCY.md
	/// before touching clients/uploadedPools or adding a new callback here: every
	/// entry point the isobus stack can call into must take this lock
	/// (std::recursive_mutex — some of these methods call each other, e.g.
	/// update_section_states -> send_section_setpoint_states), and get_clients()
	/// must keep returning a copy, not a reference (see its declaration above).
	mutable std::recursive_mutex clientsMutex;
};
