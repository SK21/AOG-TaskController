/**
 * @brief The rate source page on the VT: objects generated in code (see assign_page.hpp)
 *
 * Each object is an ISO 11783-6 binary record: object ID (2 bytes, little endian), type (1 byte),
 * then the type's attributes. Layout is in the designer pool's 480 x 480 units; the VT client
 * scales the whole pool to the terminal.
 */
#include "assign_page.hpp"

#include "AOG_TC.iop.h"

#include <array>
#include <utility>

namespace assign_page
{
	namespace
	{
		// ISO 11783-6 object types
		constexpr std::uint8_t TYPE_DATA_MASK = 1;
		constexpr std::uint8_t TYPE_SOFT_KEY_MASK = 4;
		constexpr std::uint8_t TYPE_KEY = 5;
		constexpr std::uint8_t TYPE_BUTTON = 6;
		constexpr std::uint8_t TYPE_INPUT_LIST = 10;
		constexpr std::uint8_t TYPE_OUTPUT_STRING = 11;
		constexpr std::uint8_t TYPE_OUTPUT_RECTANGLE = 14;
		constexpr std::uint8_t TYPE_LINE_ATTRIBUTES = 24;
		constexpr std::uint8_t TYPE_FILL_ATTRIBUTES = 25;

		// Same look as the designer pool: silver masks, grey keys, its text font
		constexpr std::uint8_t COLOUR_MASK = 7;
		constexpr std::uint8_t COLOUR_KEY = 8;
		constexpr std::uint8_t COLOUR_WHITE = 1;
		constexpr std::uint8_t COLOUR_BLACK = 0;
		constexpr std::uint16_t FONT = FontAttributes_23001;
		constexpr std::uint16_t NULL_ID = 0xFFFF;

		constexpr std::uint8_t STRING_TRANSPARENT = 0x01;
		constexpr std::uint8_t JUSTIFY_LEFT_MIDDLE = 0x04;
		constexpr std::uint8_t JUSTIFY_CENTRE_MIDDLE = 0x05;
		constexpr std::uint8_t LIST_ENABLED = 0x01;

		// Key codes: the designer's five navigation keys use 1-5
		constexpr std::uint8_t KEYCODE_RATE_SOURCES = 6;
		constexpr std::uint8_t KEYCODE_PREV = 7;
		constexpr std::uint8_t KEYCODE_NEXT = 8;
		constexpr std::uint8_t KEYCODE_BACK = 9;

		constexpr std::uint16_t LINE_FRAME = 30210; ///< Thin black border of the list boxes
		constexpr std::uint16_t FILL_FRAME = 30211; ///< White inside the list boxes

		// Soft key label area in the designer pool's key units (the client scales keys from ISO_DESIGNATOR_HEIGHT)
		constexpr std::uint16_t KEY_LABEL_WIDTH = 78;
		constexpr std::uint16_t KEY_LABEL_HEIGHT = 40;

		struct Placed
		{
			std::uint16_t id;
			std::int16_t x;
			std::int16_t y;
		};

		class Writer
		{
		public:
			explicit Writer(std::vector<std::uint8_t> &out) :
			  bytes(out)
			{
			}

			void u8(std::uint8_t value)
			{
				bytes.push_back(value);
			}

			void u16(std::uint16_t value)
			{
				bytes.push_back(static_cast<std::uint8_t>(value & 0xFF));
				bytes.push_back(static_cast<std::uint8_t>(value >> 8));
			}

			void header(std::uint16_t id, std::uint8_t type)
			{
				u16(id);
				u8(type);
			}

			void placed(const std::vector<Placed> &children)
			{
				for (const auto &child : children)
				{
					u16(child.id);
					u16(static_cast<std::uint16_t>(child.x));
					u16(static_cast<std::uint16_t>(child.y));
				}
			}

		private:
			std::vector<std::uint8_t> &bytes;
		};

		void output_string(Writer &w, std::uint16_t id, std::uint16_t width, std::uint16_t height, std::uint8_t justification, std::uint16_t length, std::string text, bool transparent = true)
		{
			text.resize(length, ' '); // the value is exactly `length` bytes
			w.header(id, TYPE_OUTPUT_STRING);
			w.u16(width);
			w.u16(height);
			w.u8(transparent ? COLOUR_MASK : COLOUR_WHITE); // background
			w.u16(FONT);
			w.u8(transparent ? STRING_TRANSPARENT : 0);
			w.u16(NULL_ID); // no string variable
			w.u8(justification);
			w.u16(length);
			for (char c : text)
			{
				w.u8(static_cast<std::uint8_t>(c));
			}
			w.u8(0); // macros
		}

		void data_mask(Writer &w, std::uint16_t id, std::uint16_t softKeyMask, const std::vector<Placed> &children)
		{
			w.header(id, TYPE_DATA_MASK);
			w.u8(COLOUR_MASK);
			w.u16(softKeyMask);
			w.u8(static_cast<std::uint8_t>(children.size()));
			w.u8(0); // macros
			w.placed(children);
		}

		void soft_key_mask(Writer &w, std::uint16_t id, const std::vector<std::uint16_t> &keys)
		{
			w.header(id, TYPE_SOFT_KEY_MASK);
			w.u8(COLOUR_KEY);
			w.u8(static_cast<std::uint8_t>(keys.size()));
			w.u8(0); // macros
			for (auto key : keys)
			{
				w.u16(key);
			}
		}

		void key(Writer &w, std::uint16_t id, std::uint8_t keyCode, std::uint16_t labelId)
		{
			w.header(id, TYPE_KEY);
			w.u8(COLOUR_KEY);
			w.u8(keyCode);
			w.u8(1); // one child: the label
			w.u8(0); // macros
			w.placed({ { labelId, 1, static_cast<std::int16_t>((ISO_DESIGNATOR_HEIGHT - KEY_LABEL_HEIGHT) / 2) } });
		}

		void button(Writer &w, std::uint16_t id, std::uint16_t width, std::uint16_t height, std::uint8_t keyCode, const std::vector<Placed> &children)
		{
			w.header(id, TYPE_BUTTON);
			w.u16(width);
			w.u16(height);
			w.u8(COLOUR_KEY); // background
			w.u8(COLOUR_BLACK); // border
			w.u8(keyCode);
			w.u8(0); // options: momentary
			w.u8(static_cast<std::uint8_t>(children.size()));
			w.u8(0); // macros
			w.placed(children);
		}

		void line_attributes(Writer &w, std::uint16_t id, std::uint8_t colour, std::uint8_t width)
		{
			w.header(id, TYPE_LINE_ATTRIBUTES);
			w.u8(colour);
			w.u8(width);
			w.u16(0xFFFF); // solid
			w.u8(0); // macros
		}

		void fill_attributes(Writer &w, std::uint16_t id, std::uint8_t colour)
		{
			w.header(id, TYPE_FILL_ATTRIBUTES);
			w.u8(2); // fill with the fill colour
			w.u8(colour);
			w.u16(NULL_ID); // no pattern
			w.u8(0); // macros
		}

		void output_rectangle(Writer &w, std::uint16_t id, std::uint16_t width, std::uint16_t height, std::uint16_t lineAttributes, std::uint16_t fillAttributes)
		{
			w.header(id, TYPE_OUTPUT_RECTANGLE);
			w.u16(lineAttributes);
			w.u16(width);
			w.u16(height);
			w.u8(0); // no sides suppressed
			w.u16(fillAttributes);
			w.u8(0); // macros
		}

		void input_list(Writer &w, std::uint16_t id, std::uint16_t width, std::uint16_t height, const std::array<std::uint16_t, LIST_SLOTS> &items)
		{
			w.header(id, TYPE_INPUT_LIST);
			w.u16(width);
			w.u16(height);
			w.u16(NULL_ID); // no number variable
			w.u8(0xFF); // nothing selected until the page fills it
			w.u8(static_cast<std::uint8_t>(items.size()));
			w.u8(LIST_ENABLED);
			w.u8(0); // macros
			for (auto item : items)
			{
				w.u16(item);
			}
		}

		// Layout, 480 x 480
		constexpr std::int16_t COL_ELEMENT = 8;
		constexpr std::uint16_t W_ELEMENT = 172;
		constexpr std::int16_t COL_TYPE = 184;
		constexpr std::uint16_t W_TYPE = 72;
		constexpr std::int16_t COL_LIST = 260;
		constexpr std::uint16_t W_LIST = 124;
		constexpr std::int16_t COL_NOW = 388;
		constexpr std::uint16_t W_NOW = 88;
		constexpr std::int16_t ROW_TOP = 78;
		constexpr std::int16_t ROW_PITCH = 44;
		constexpr std::uint16_t ROW_HEIGHT = 36;

		constexpr std::int16_t LIST_INSET = 4; ///< The list sits inside its white box
		constexpr std::uint16_t W_LIST_INNER = W_LIST - 2 * LIST_INSET;
		constexpr std::uint16_t W_ITEM = LEN_ITEM * 12 + 12; ///< Wider than the list: clipped in the box, whole in the drop-down

		constexpr std::uint16_t LABEL_RATE_SOURCES = 30020;
		constexpr std::uint16_t KEY_LABEL_PREV = 30021;
		constexpr std::uint16_t KEY_LABEL_NEXT = 30022;
		constexpr std::uint16_t KEY_LABEL_BACK = 30023;

		// "Rate sources >" on the Implement mask: the free band right of the labels, above the hint line
		constexpr std::int16_t BUTTON_X = 260;
		constexpr std::int16_t BUTTON_Y = 382;
		constexpr std::uint16_t BUTTON_WIDTH = 200;
		constexpr std::uint16_t BUTTON_HEIGHT = 44;

		// Data mask record: ID (2), type (1), background (1), soft key mask (2), object count (1), macro count (1), objects (6 each)
		constexpr std::size_t MASK_OBJECT_COUNT = 6;
		constexpr std::size_t MASK_OBJECTS = 8;

		// The Implement mask's record starts with: ID 1003, type 1, background 7, soft key mask 4000.
		// A child (the button) is added to it; anything else means the designer pool changed.
		std::size_t find_implement_mask(const std::vector<std::uint8_t> &pool, std::size_t &matches)
		{
			const std::array<std::uint8_t, 6> signature = {
				static_cast<std::uint8_t>(DataMask_Implement & 0xFF), static_cast<std::uint8_t>(DataMask_Implement >> 8),
				TYPE_DATA_MASK, COLOUR_MASK,
				static_cast<std::uint8_t>(MainPageSoftKeys & 0xFF), static_cast<std::uint8_t>(MainPageSoftKeys >> 8)
			};
			std::size_t found = 0;
			matches = 0;
			for (std::size_t i = 0; i + signature.size() <= pool.size(); i++)
			{
				bool same = true;
				for (std::size_t j = 0; (j < signature.size()) && same; j++)
				{
					same = (pool[i + j] == signature[j]);
				}
				if (same)
				{
					found = i;
					matches++;
				}
			}
			return found;
		}
	}

	bool add_to_pool(std::vector<std::uint8_t> &pool, std::string &error)
	{
		bool added = false;
		std::size_t matches = 0;
		const std::size_t implementMask = find_implement_mask(pool, matches);
		if (matches != 1)
		{
			error = "the Implement mask's record was found " + std::to_string(matches) + " times (expected once): AOG_TC.iop changed";
		}
		else
		{
			std::vector<std::uint8_t> objects;
			Writer w(objects);

			// The button on the Implement mask that opens the page
			button(w, Button_RateSources, BUTTON_WIDTH, BUTTON_HEIGHT, KEYCODE_RATE_SOURCES, { { LABEL_RATE_SOURCES, 8, 8 } });
			output_string(w, LABEL_RATE_SOURCES, BUTTON_WIDTH - 16, BUTTON_HEIGHT - 16, JUSTIFY_CENTRE_MIDDLE, 14, "Rate sources >");

			// The page's soft keys: Back to Implement; Prev and Next only when there is more than one page
			soft_key_mask(w, SoftKeys_Assign, { Key_Back });
			soft_key_mask(w, SoftKeys_AssignPaged, { Key_Prev, Key_Next, Key_Back });
			key(w, Key_Prev, KEYCODE_PREV, KEY_LABEL_PREV);
			key(w, Key_Next, KEYCODE_NEXT, KEY_LABEL_NEXT);
			key(w, Key_Back, KEYCODE_BACK, KEY_LABEL_BACK);
			output_string(w, KEY_LABEL_PREV, KEY_LABEL_WIDTH, KEY_LABEL_HEIGHT, JUSTIFY_CENTRE_MIDDLE, 4, "Prev");
			output_string(w, KEY_LABEL_NEXT, KEY_LABEL_WIDTH, KEY_LABEL_HEIGHT, JUSTIFY_CENTRE_MIDDLE, 4, "Next");
			output_string(w, KEY_LABEL_BACK, KEY_LABEL_WIDTH, KEY_LABEL_HEIGHT, JUSTIFY_CENTRE_MIDDLE, 4, "Back");

			// The mask
			std::vector<Placed> children = {
				{ Str_Title, 8, 8 },
				{ Str_Page, 320, 8 },
				{ Str_Header, 8, 44 },
				{ Str_Warning, 8, 436 },
			};
			for (std::uint8_t row = 0; row < ROWS; row++)
			{
				const auto y = static_cast<std::int16_t>(ROW_TOP + row * ROW_PITCH);
				children.push_back({ row_element(row), COL_ELEMENT, y });
				children.push_back({ row_type(row), COL_TYPE, y });
				children.push_back({ row_frame(row), COL_LIST, y }); // drawn before (under) the list
				children.push_back({ row_list(row), static_cast<std::int16_t>(COL_LIST + LIST_INSET), y });
				children.push_back({ row_now(row), COL_NOW, y });
			}
			data_mask(w, DataMask_Assign, SoftKeys_Assign, children);

			output_string(w, Str_Title, 300, 32, JUSTIFY_LEFT_MIDDLE, LEN_TITLE, "RATE SOURCES");
			output_string(w, Str_Page, 152, 32, JUSTIFY_LEFT_MIDDLE, LEN_PAGE, "");
			output_string(w, Str_Header, 464, 28, JUSTIFY_LEFT_MIDDLE, LEN_HEADER, "Element        Rate  Source    Src:Now");
			output_string(w, Str_Warning, 464, 36, JUSTIFY_LEFT_MIDDLE, LEN_WARNING, "");

			// Each list sits in a white box with a thin border, the usual look of an input field
			line_attributes(w, LINE_FRAME, COLOUR_BLACK, 1);
			fill_attributes(w, FILL_FRAME, COLOUR_WHITE);
			std::array<std::uint16_t, LIST_SLOTS> emptySlots;
			emptySlots.fill(NULL_ID);
			for (std::uint8_t row = 0; row < ROWS; row++)
			{
				output_string(w, row_element(row), W_ELEMENT, ROW_HEIGHT, JUSTIFY_LEFT_MIDDLE, LEN_ELEMENT, "");
				output_string(w, row_type(row), W_TYPE, ROW_HEIGHT, JUSTIFY_LEFT_MIDDLE, LEN_TYPE, "");
				output_rectangle(w, row_frame(row), W_LIST, ROW_HEIGHT, LINE_FRAME, FILL_FRAME);
				input_list(w, row_list(row), W_LIST_INNER, ROW_HEIGHT, emptySlots);
				output_string(w, row_now(row), W_NOW, ROW_HEIGHT, JUSTIFY_LEFT_MIDDLE, LEN_NOW, "");
			}

			// List items, shared by every row; source names are filled in at run time
			output_string(w, Item_None, W_ITEM, ROW_HEIGHT, JUSTIFY_LEFT_MIDDLE, LEN_ITEM, "None");
			output_string(w, Item_Auto, W_ITEM, ROW_HEIGHT, JUSTIFY_LEFT_MIDDLE, LEN_ITEM, "Auto");
			for (std::uint8_t source = 1; source <= MAX_SOURCES; source++)
			{
				output_string(w, item_source(source), W_ITEM, ROW_HEIGHT, JUSTIFY_LEFT_MIDDLE, LEN_ITEM, std::to_string(source));
			}

			// Add the button as one more child of the Implement mask (inserted before its macro list)
			const std::uint8_t objectCount = pool[implementMask + MASK_OBJECT_COUNT];
			const std::size_t insertAt = implementMask + MASK_OBJECTS + static_cast<std::size_t>(objectCount) * 6;
			if ((objectCount == 0xFF) || (insertAt > pool.size()))
			{
				error = "the Implement mask's object list is not as expected: AOG_TC.iop changed";
			}
			else
			{
				const std::array<std::uint8_t, 6> child = {
					static_cast<std::uint8_t>(Button_RateSources & 0xFF), static_cast<std::uint8_t>(Button_RateSources >> 8),
					static_cast<std::uint8_t>(BUTTON_X & 0xFF), static_cast<std::uint8_t>(BUTTON_X >> 8),
					static_cast<std::uint8_t>(BUTTON_Y & 0xFF), static_cast<std::uint8_t>(BUTTON_Y >> 8)
				};
				pool[implementMask + MASK_OBJECT_COUNT] = static_cast<std::uint8_t>(objectCount + 1);
				pool.insert(pool.begin() + static_cast<std::ptrdiff_t>(insertAt), child.begin(), child.end());
				pool.insert(pool.end(), objects.begin(), objects.end());
				added = true;
			}
		}
		return added;
	}
}
