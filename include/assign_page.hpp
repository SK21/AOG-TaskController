/**
 * @brief The rate source page on the VT: objects generated in code and appended to the designer pool
 *
 * The page lets the operator choose, per implement rate element, which rate source drives it
 * (today RateController's products). Its objects are built here rather than in ISO-Designer and
 * appended to AOG_TC.iop at start; a "Rate sources >" button is added to the Implement mask.
 * Object IDs 30000-30999 are reserved for generated objects.
 */
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace assign_page
{
	constexpr std::uint16_t DataMask_Assign = 30000;
	constexpr std::uint16_t SoftKeys_Assign = 30001; ///< Back only (one page)
	constexpr std::uint16_t SoftKeys_AssignPaged = 30003; ///< Prev, Next, Back (more than ROWS elements)

	constexpr std::uint16_t Button_RateSources = 30014; ///< On DataMask_Implement: opens the page (a sub-page of Implement)
	constexpr std::uint16_t Key_Prev = 30011;
	constexpr std::uint16_t Key_Next = 30012;
	constexpr std::uint16_t Key_Back = 30013;

	constexpr std::uint16_t Str_Title = 30030;
	constexpr std::uint16_t Str_Page = 30031;
	constexpr std::uint16_t Str_Header = 30032;
	constexpr std::uint16_t Str_Warning = 30033;

	constexpr std::uint8_t ROWS = 8; ///< Element rows per page (D5)
	constexpr std::uint16_t row_element(std::uint8_t row) { return static_cast<std::uint16_t>(30100 + row); }
	constexpr std::uint16_t row_type(std::uint8_t row) { return static_cast<std::uint16_t>(30110 + row); }
	constexpr std::uint16_t row_list(std::uint8_t row) { return static_cast<std::uint16_t>(30120 + row); }
	constexpr std::uint16_t row_now(std::uint8_t row) { return static_cast<std::uint16_t>(30130 + row); }
	constexpr std::uint16_t row_frame(std::uint8_t row) { return static_cast<std::uint16_t>(30140 + row); } ///< White bordered box behind the list

	/// Input list slots, the same in every row: the selected index always means the same choice.
	/// Slots that don't apply to a row (sources of another type, empty rows) hold NULL.
	constexpr std::uint8_t SLOT_NONE = 0;
	constexpr std::uint8_t SLOT_AUTO = 1;
	constexpr std::uint8_t SLOT_FIRST_SOURCE = 2; ///< Slot of source 1
	constexpr std::uint8_t MAX_SOURCES = 5;
	constexpr std::uint8_t LIST_SLOTS = SLOT_FIRST_SOURCE + MAX_SOURCES;

	constexpr std::uint16_t Item_None = 30200;
	constexpr std::uint16_t Item_Auto = 30201;
	constexpr std::uint16_t item_source(std::uint8_t source) { return static_cast<std::uint16_t>(30201 + source); } ///< source 1-5

	// String lengths (characters) the page code must not exceed
	constexpr std::uint16_t LEN_TITLE = 24;
	constexpr std::uint16_t LEN_PAGE = 12;
	constexpr std::uint16_t LEN_HEADER = 38;
	constexpr std::uint16_t LEN_WARNING = 38;
	constexpr std::uint16_t LEN_ELEMENT = 14;
	constexpr std::uint16_t LEN_TYPE = 6;
	constexpr std::uint16_t LEN_NOW = 7; ///< 12 px font: 7 characters fit the 88 px column
	/// List items hold "n " + RateController's longest product name (15). The box shows what fits and clips
	/// the rest; the VT's drop-down draws each item at its own width, so the whole name shows there.
	constexpr std::uint16_t LEN_ITEM = 17;

	/// @brief Appends the page's objects to the designer pool and adds Button_RateSources to DataMask_Implement
	/// @param pool The designer pool (AOG_TC.iop bytes), changed only on success
	/// @param error Why it failed (the designer pool changed in a way this code doesn't recognise)
	/// @returns true when the page was added
	bool add_to_pool(std::vector<std::uint8_t> &pool, std::string &error);
}
