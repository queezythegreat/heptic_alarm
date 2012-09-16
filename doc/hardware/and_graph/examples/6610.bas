' ------------------------------------------------------------------------------
'                      TEST NOKIA6100
' ------------------------------------------------------------------------------
$lib "lcd-pcf8833.lbx"
$regfile = "m16def.dat"
$crystal = 16000000
$hwstack = 128
$swstack = 128
$framesize = 128
Config Graphlcd = Color , Controlport = Portc , Cs = 0 , Rs = 3 , Scl = 1 , Sda = 2

Config Porta.0 = Output
Ledy Alias Porta.0
Dim Z As Byte
Dim Ya As Byte
Set Ledy

Const Blue = &B00000011
Const Yellow = &B11111100
Const Red = &B11100000
Const Green = &B00011100
Const Black = &B00000000
Const White = &B11111111
Const Brightgreen = &B00111110
Const Lightgreen = &B01111100
Const Darkgreen = &B00010100
Const Darkred = &B10100000
Const Darkblue = &B00000010
Const Brightblue = &B00011111
Const Orange = &B11111000

'Lcd_write 0 , &H36
'Lcd_write 1 , &H48
'Lcd_write 0 , &H21 'Inversion_on

Glcdcmd &H36                                                'Mem_control
Glcddata &H98                                               'My=1 Mx=0 V=0 Lao=1 Rbg=1 "10011000"
Glcdcmd &H21                                                'Display inversion on
'Glcdcmd &H20                                                'Display inversion off

For Z = 1 To 5
Reset Ledy
Waitms 255
Set Ledy
Waitms 255
Next

Cls
Setfont Color16x16
Lcdat 10 , 2 , "AND-TECH" , Green , White
Pocz:
Ya = Ya + 1
Setfont Color16x16
Lcdat 44 , 58 , Ya , Brightgreen , Darkblue
Box(0 , 80) -(132 , 90) , Red
Circle(110 , 58), 10 , orange
Pset 1 , 1 , Red
Wait 1
Goto Pocz


End
$include "color16x16.font"


