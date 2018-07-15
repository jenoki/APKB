#ifndef KEYBOARD_H
    #define KEYBOARD_H

//  #define APKB
    #define LRKB

    #define APKB_RELASE_NO 0x472

    #define KEYBOARD_LEN    0x05
    #define MOUSE_LEN   0x05

    #define WHEEL_UP	0x01
    #define WHEEL_DOWN	0xFF
    #define WHEEL_RIGHT	0x01
    #define WHEEL_LEFT	0xFF
    #define MOUSE_MOVE_LEFT	0xE0
    #define MOUSE_MOVE_RIGHT	0x20

    #define MOUSE_BUTTON_LEFT   0x1
    #define MOUSE_BUTTON_CENTER 0x2
    #define MOUSE_BUTTON_RIGHT  0x4

    #define MOUSE_RPT_IDX_BTN   0
    #define MOUSE_RPT_IDX_MOVE  1
    #define MOUSE_RPT_IDX_ROTATE 3
    #define MOUSE_RPT_IDX_TILT  4
    #define KEYBOARD_RPT_IDX_M  0

    /* Key bit mask patterns*/
    #define AKB00   0x0001
    #define AKB01   0x0002
    #define AKB02   0x0004
    #define AKB03   0x0008
    #define AKB04   0x0010
    #define AKB05   0x0020
    #define AKB06   0x0040
    #define AKB07   0x0080
    #define AKB08   0x0100
    #define AKB09   0x0200
    #define AKB10   0x0400
    #define AKB11   0x0800
    #define AKB12   0x1000
    #define AKB13   0x2000
    #define AKB14   0x4000
    #define AKB15   0x8000
	
//HID key codes HID 1.11
//alphabet
    #define HID_a   0x04
    #define HID_b   0x05
    #define HID_c   0x06
    #define HID_d   0x07
    #define HID_e   0x08
    #define HID_f   0x09
    #define HID_g   0x0A
    #define HID_h   0x0B
    #define HID_i   0x0C
    #define HID_j   0x0D
    #define HID_k   0x0E
    #define HID_l   0x0F
    #define HID_m   0x10
    #define HID_n   0x11
    #define HID_o   0x12
    #define HID_p   0x13
    #define HID_q   0x14
    #define HID_r   0x15
    #define HID_s   0x16
    #define HID_t   0x17
    #define HID_u   0x18
    #define HID_v   0x19
    #define HID_w   0x1A
    #define HID_x   0x1B
    #define HID_y   0x1C
    #define HID_z   0x1D
//numeric
    #define HID_1   0x1E
    #define HID_2   0x1F
    #define HID_3   0x20
    #define HID_4   0x21
    #define HID_5   0x22
    #define HID_6   0x23
    #define HID_7   0x24
    #define HID_8   0x25
    #define HID_9   0x26
    #define HID_0   0x27
//control
    #define HIDBS   0x2A
    #define HIDTAB  0x2B
    #define HID_AT  0x2F
    #define HID_LA  0x36    //Left Allow '<'
    #define HID_CM  0x36    //Comma ','
    #define HID_RA  0x37    //Right Allow '>'
    #define HID_DT  0x37    //Period '.'
    #define HID_QS  0x38
    #define HID_UA  0x2E    //Up Arrow '^'
    #define HID_RB  0x32    //Right brace ']'
    #define HID_SL  0x38    //Slash '/'
//cursor
    #define HIDUP   0x52
    #define HIDDOWN 0x51
    #define HIDLEFT 0x50
    #define HIDRIGHT    0x4F
//Modifier codes
    #define HIDGUIM 0x08
    #define HIDALTM 0x04
    #define HIDSFTM 0x02
    #define HIDCTLM 0x01

    /* Keyboard Layouts for APKB Board 4.0
	B00	B01	B02	B03
	B04	B05	B06	B07
	B08	B09	B10	B11
	B12	B13	B14	B15
    */
    #define MOUSE_BIT_MASK 0x3720


#if defined(APKB) && defined(LRKB)
#error do not use APKB and LRKB same time.
#elif !(defined(APKB) || defined(LRKB))
#error use APKB or LRKB either.
#endif

#endif
