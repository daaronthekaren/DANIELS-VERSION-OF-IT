#pragma once
// Override board-level USB descriptor strings.
// This file is force-included via build_flags so it runs after all -D
// command-line defines (including the board's default "Pico 2"), giving
// these definitions the final say.
#ifdef USB_PRODUCT
#undef USB_PRODUCT
#endif
#define USB_PRODUCT "Micro-Manipulator"

#ifdef USB_MANUFACTURER
#undef USB_MANUFACTURER
#endif
#define USB_MANUFACTURER ""
