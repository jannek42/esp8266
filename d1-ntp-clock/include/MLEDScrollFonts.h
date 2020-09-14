const char matrix_fonts[] PROGMEM = {
  0x3C,0x66,0x6E,0x76,0x66,0x66,0x3C,0x00,  // 0x00   0  (0)
  0x18,0x18,0x38,0x18,0x18,0x18,0x7E,0x00,  // 0x01   1  (1)
  0x3C,0x66,0x06,0x0C,0x30,0x60,0x7E,0x00,  // 0x02   2  (2)
  0x3C,0x66,0x06,0x1C,0x06,0x66,0x3C,0x00,  // 0x03   3  (3)
  0x06,0x0E,0x16,0x26,0x7F,0x06,0x06,0x00,  // 0x04   4  (4)
  0x7E,0x60,0x7C,0x06,0x06,0x66,0x3C,0x00,  // 0x05   5  (5)
  0x3C,0x66,0x60,0x7C,0x66,0x66,0x3C,0x00,  // 0x06   6  (6)
  0x7E,0x66,0x0C,0x18,0x18,0x18,0x18,0x00,  // 0x07   7  (7)
  0x3C,0x66,0x66,0x3C,0x66,0x66,0x3C,0x00,  // 0x08   8  (8)
  0x3C,0x66,0x66,0x3E,0x06,0x66,0x3C,0x00,  // 0x09   9  (9) 
  0x18,0x3C,0x66,0x7E,0x66,0x66,0x66,0x00,  // 0x0A  10  (A) 
  0x7C,0x66,0x66,0x7C,0x66,0x66,0x7C,0x00,  // 0x0B  11  (B) 
  0x3C,0x66,0x60,0x60,0x60,0x66,0x3C,0x00,  // 0x0C  12  (C)
  0x78,0x6C,0x66,0x66,0x66,0x6C,0x78,0x00,  // 0x0D  13  (D)
  0x7E,0x60,0x60,0x78,0x60,0x60,0x7E,0x00,  // 0x0E  14  (E)
  0x7E,0x60,0x60,0x78,0x60,0x60,0x60,0x00,  // 0x0F  15  (F)
	0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0x10  16  Row 1 on
  0x00,0x00,0xFF,0x00,0x00,0x00,0x00,0x00,  // 0x12  18  Row 3 on
  0x00,0x00,0x00,0xFF,0x00,0x00,0x00,0x00,  // 0x13  19  Row 4 on
  0x00,0x00,0x00,0x00,0xFF,0x00,0x00,0x00,  // 0x14  20  Row 5 on
  0x00,0x00,0x00,0x00,0x00,0xFF,0x00,0x00,  // 0x15  21  Row 6 on
  0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0x00,  // 0x16  22  Row 7 on
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,  // 0x17  23  Row 8 on
  0x00,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,  // 0x11  17  Row 2 on
  0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,  // 0x18  24  Col 1 on
  0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02,  // 0x19  25  Col 2 on
  0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x04,  // 0x1A  26  Col 3 on
  0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,  // 0x1B  27  Col 4 on
  0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,  // 0x1C  28  Col 5 on
  0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,  // 0x1D  29  Col 6 on
  0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x40,  // 0x1E  30  Col 7 on
  0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,  // 0x1F  31  Col 8 on
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0x20  32  ( )
  0x18,0x18,0x18,0x18,0x00,0x00,0x18,0x00,  // 0x21  33  (!)
  0x6C,0x6c,0x24,0x00,0x00,0x00,0x00,0x00,  // 0x22  34  (")
  0x66,0x66,0xFF,0x66,0xFF,0x66,0x66,0x00,  // 0x23  35  (#)
  0x18,0x3E,0x60,0x3C,0x06,0x7C,0x18,0x00,  // 0x24  36  ($)
  0x62,0x66,0x0C,0x18,0x30,0x66,0x46,0x00,  // 0x25  37  (%)
  0x3C,0x66,0x3C,0x38,0x67,0x66,0x3F,0x00,  // 0x26  38  (&)
  0x18,0x18,0x30,0x00,0x00,0x00,0x00,0x00,  // 0x27  39  (')
  0x0C,0x18,0x30,0x30,0x30,0x18,0x0C,0x00,  // 0x28  40  (()
  0x30,0x18,0x0C,0x0C,0x0C,0x18,0x30,0x00,  // 0x29  41  ())
  0x00,0x66,0x3C,0xFF,0x3C,0x66,0x00,0x00,  // 0x2A  42  (*)
  0x00,0x18,0x18,0x7E,0x18,0x18,0x00,0x00,  // 0x2B  43  (+)
  0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x30,  // 0x2C  44  (,)
  0x00,0x00,0x00,0x7E,0x00,0x00,0x00,0x00,  // 0x2D  45  (-)
  0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x00,  // 0x2E  46  (.)
  0x00,0x03,0x06,0x0C,0x18,0x30,0x60,0x00,  // 0x2F  47  (/)
  0x3C,0x66,0x6E,0x76,0x66,0x66,0x3C,0x00,  // 0x30  48  (0)
  0x18,0x18,0x38,0x18,0x18,0x18,0x7E,0x00,  // 0x31  49  (1)
  0x3C,0x66,0x06,0x0C,0x30,0x60,0x7E,0x00,  // 0x32  50  (2)
  0x3C,0x66,0x06,0x1C,0x06,0x66,0x3C,0x00,  // 0x33  51  (3)
  0x06,0x0E,0x16,0x26,0x7F,0x06,0x06,0x00,  // 0x34  52  (4)
  0x7E,0x60,0x7C,0x06,0x06,0x66,0x3C,0x00,  // 0x35  53  (5)
  0x3C,0x66,0x60,0x7C,0x66,0x66,0x3C,0x00,  // 0x36  54  (6)
  0x7E,0x66,0x0C,0x18,0x18,0x18,0x18,0x00,  // 0x37  55  (7)
  0x3C,0x66,0x66,0x3C,0x66,0x66,0x3C,0x00,  // 0x38  56  (8)
  0x3C,0x66,0x66,0x3E,0x06,0x66,0x3C,0x00,  // 0x39  57  (9)
  0x00,0x00,0x18,0x00,0x00,0x18,0x00,0x00,  // 0x3A  58  (:)
  0x00,0x00,0x18,0x00,0x00,0x18,0x18,0x30,  // 0x3B  59  (;)
  0x0E,0x18,0x30,0x60,0x30,0x18,0x0E,0x00,  // 0x3C  60  (<)
  0x00,0x00,0x7E,0x00,0x7E,0x00,0x00,0x00,  // 0x3D  61  (=)
  0x70,0x18,0x0C,0x06,0x0C,0x18,0x70,0x00,  // 0x3E  62  (>)
  0x3C,0x66,0x06,0x0C,0x18,0x00,0x18,0x00,  // 0x3F  63  (?)
  0x3C,0x66,0x6E,0x6E,0x60,0x62,0x3C,0x00,  // 0x40  64  (@)
  0x18,0x3C,0x66,0x7E,0x66,0x66,0x66,0x00,  // 0x41  65  (A)
  0x7C,0x66,0x66,0x7C,0x66,0x66,0x7C,0x00,  // 0x42  66  (B)
  0x3C,0x66,0x60,0x60,0x60,0x66,0x3C,0x00,  // 0x43  67  (C)
  0x78,0x6C,0x66,0x66,0x66,0x6C,0x78,0x00,  // 0x44  68  (D)
  0x7E,0x60,0x60,0x78,0x60,0x60,0x7E,0x00,  // 0x45  69  (E)
  0x7E,0x60,0x60,0x78,0x60,0x60,0x60,0x00,  // 0x46  70  (F)
  0x3C,0x66,0x60,0x6E,0x66,0x66,0x3C,0x00,  // 0x47  71  (G)
  0x66,0x66,0x66,0x7E,0x66,0x66,0x66,0x00,  // 0x48  72  (H)
  0x3C,0x18,0x18,0x18,0x18,0x18,0x3C,0x00,  // 0x49  73  (I)
  0x1E,0x0C,0x0C,0x0C,0x0C,0x6C,0x38,0x00,  // 0x4A  74  (J)
  0x66,0x6C,0x78,0x70,0x78,0x6C,0x66,0x00,  // 0x4B  75  (K)
  0x60,0x60,0x60,0x60,0x60,0x60,0x7E,0x00,  // 0x4C  76  (L)
  0x63,0x77,0x7F,0x6B,0x63,0x63,0x63,0x00,  // 0x4D  77  (M)
  0x66,0x76,0x7E,0x7E,0x6E,0x66,0x66,0x00,  // 0x4E  78  (N)
  0x3C,0x66,0x66,0x66,0x66,0x66,0x3C,0x00,  // 0x4F  79  (O)
  0x7C,0x66,0x66,0x7C,0x60,0x60,0x60,0x00,  // 0x50  80  (P)
  0x3C,0x66,0x66,0x66,0x66,0x3C,0x0E,0x00,  // 0x51  81  (Q)
  0x7C,0x66,0x66,0x7C,0x78,0x6C,0x66,0x00,  // 0x52  82  (R)
  0x3C,0x66,0x60,0x3C,0x06,0x66,0x3C,0x00,  // 0x53  83  (S)
  0x7E,0x18,0x18,0x18,0x18,0x18,0x18,0x00,  // 0x54  84  (T)
  0x66,0x66,0x66,0x66,0x66,0x66,0x3C,0x00,  // 0x55  85  (U)
  0x66,0x66,0x66,0x66,0x66,0x3C,0x18,0x00,  // 0x56  86  (V)
  0x63,0x63,0x63,0x6B,0x7F,0x77,0x63,0x00,  // 0x57  87  (W)
  0x66,0x66,0x3C,0x18,0x3C,0x66,0x66,0x00,  // 0x58  88  (X)
  0x66,0x66,0x66,0x3C,0x18,0x18,0x18,0x00,  // 0x59  89  (Y)
  0x7E,0x06,0x0C,0x18,0x30,0x60,0x7E,0x00,  // 0x5A  90  (Z)
  0x3C,0x30,0x30,0x30,0x30,0x30,0x3C,0x00,  // 0x5B  91  ([)
  0x00,0x60,0x30,0x18,0x0C,0x06,0x03,0x00,  // 0x5C  92  (\)
  0x3C,0x0C,0x0C,0x0C,0x0C,0x0C,0x3C,0x00,  // 0x5D  93  (])
  0x18,0x24,0x42,0x00,0x00,0x00,0x00,0x00,  // 0x5E  94  (^)
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,  // 0x5F  95  (_)
  0x18,0x18,0x0C,0x00,0x00,0x00,0x00,0x00,  // 0x60  96  (`)
  0x00,0x00,0x3C,0x06,0x3E,0x66,0x3E,0x00,  // 0x61  97  (a)
  0x00,0x60,0x60,0x7C,0x66,0x66,0x7C,0x00,  // 0x62  98  (b)
  0x00,0x00,0x3C,0x60,0x60,0x60,0x3C,0x00,  // 0x63  99  (c)
  0x00,0x06,0x06,0x3E,0x66,0x66,0x3E,0x00,  // 0x64 100  (d)
  0x00,0x00,0x3C,0x66,0x7E,0x60,0x3C,0x00,  // 0x65 101  (e)
  0x00,0x0E,0x18,0x3E,0x18,0x18,0x18,0x00,  // 0x66 102  (f)
  0x00,0x00,0x3E,0x66,0x66,0x3E,0x06,0x7C,  // 0x67 103  (g)
  0x00,0x60,0x60,0x7C,0x66,0x66,0x66,0x00,  // 0x68 104  (h)
  0x00,0x18,0x00,0x38,0x18,0x18,0x3C,0x00,  // 0x69 105  (i)
  0x00,0x06,0x00,0x06,0x06,0x06,0x06,0x3C,  // 0x6A 106  (j)
  0x00,0x60,0x60,0x6C,0x78,0x6C,0x66,0x00,  // 0x6B 107  (k)
  0x00,0x38,0x18,0x18,0x18,0x18,0x3C,0x00,  // 0x6C 108  (l)
  0x00,0x00,0x66,0x7F,0x7F,0x6B,0x63,0x00,  // 0x6D 109  (m)
  0x00,0x00,0x7C,0x66,0x66,0x66,0x66,0x00,  // 0x6E 110  (n)
  0x00,0x00,0x3C,0x66,0x66,0x66,0x3C,0x00,  // 0x6F 111  (o)
  0x00,0x00,0x7C,0x66,0x66,0x7C,0x60,0x60,  // 0x70 112  (p)
  0x00,0x00,0x3E,0x66,0x66,0x3E,0x06,0x06,  // 0x71 113  (q)
  0x00,0x00,0x7C,0x66,0x60,0x60,0x60,0x00,  // 0x72 114  (r)
  0x00,0x00,0x3E,0x60,0x3C,0x06,0x7C,0x00,  // 0x73 115  (s)
  0x00,0x18,0x7E,0x18,0x18,0x18,0x0E,0x00,  // 0x74 116  (t)
  0x00,0x00,0x66,0x66,0x66,0x66,0x3E,0x00,  // 0x75 117  (u)
  0x00,0x00,0x66,0x66,0x66,0x3C,0x18,0x00,  // 0x76 118  (v)
  0x00,0x00,0x63,0x6B,0x7F,0x3E,0x36,0x00,  // 0x77 119  (w)
  0x00,0x00,0x66,0x3C,0x18,0x3C,0x66,0x00,  // 0x78 120  (x)
  0x00,0x00,0x66,0x66,0x66,0x3E,0x0C,0x78,  // 0x79 121  (y)
  0x00,0x00,0x7E,0x0C,0x18,0x30,0x7E,0x00,  // 0x7A 122  (z)
  0x1C,0x30,0x30,0x60,0x30,0x30,0x1C,0x00,  // 0x7B 123  ({)
  0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x00,  // 0x7C 124  (|)
  0x38,0x0C,0x0C,0x06,0x0C,0x0C,0x38,0x00,  // 0x7D 125  (})
  0x3C,0x42,0xA5,0x81,0xA5,0x99,0x42,0x3C,  // 0x7E 126  (~)
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0x7F 127  (DEL)
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0x81 128  
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0x82 129  
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0x83 130  
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0x84 131  
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0x85 132  
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0x80 133  
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0x86 134  
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0x87 135  
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0x88 136  
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0x89 137  
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0x8A 138  
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0x8B 139  
  0x08,0x3C,0x60,0x3C,0x06,0x66,0x3C,0x00,  // 0x8C 140  (Ś) WIN
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0x8D 141  
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0x8E 142  
  0x18,0x7E,0x0C,0x18,0x30,0x60,0x7E,0x00,  // 0x8F 143  (Ź) WIN
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0x90 144  
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0x91 145  
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0x92 146  
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0x93 147  
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0x94 148  
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0x95 149  
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0x96 150  
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0x97 151  
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0x98 152  
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0x99 153  
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0x9A 154  
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0x9B 155  
  0x04,0x08,0x3E,0x60,0x3C,0x06,0x7C,0x00,  // 0x9C 156  (ś) WIN
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0x9D 157  
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0x9E 158  
  0x04,0x08,0x7E,0x0C,0x18,0x30,0x7E,0x00,  // 0x9F 159  (ź) WIN
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0xA0 160  
  0x18,0x3C,0x66,0x7E,0x66,0x66,0x66,0x04,  // 0xA1 161  (Ą) ISO
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0xA2 162  
  0x60,0x60,0x68,0x70,0x60,0x60,0x7E,0x00,  // 0xA3 163  (Ł) ISO/WIN
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0xA4 164  
  0x18,0x3C,0x66,0x7E,0x66,0x66,0x66,0x04,  // 0xA5 165  (Ą) WIN
  0x08,0x3C,0x60,0x3C,0x06,0x66,0x3C,0x00,  // 0xA6 166  (Ś) ISO
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0xA7 167  
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0xA8 168  
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0xA9 169  
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0xAA 170  
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0xAB 171  
  0x18,0x7E,0x0C,0x18,0x30,0x60,0x7E,0x00,  // 0xAC 172  (Ź) ISO
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0xAD 173  
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0xAE 174  
  0x08,0x7E,0x0C,0x18,0x30,0x60,0x7E,0x00,  // 0xAF 175  (Ż) ISO/WIN
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0xB0 176  
  0x00,0x00,0x3C,0x06,0x3E,0x66,0x3E,0x04,  // 0xB1 177  (ą) ISO
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0xB2 178  
  0x00,0x38,0x1C,0x18,0x38,0x18,0x3C,0x00,  // 0xB3 179  (ł) ISO/WIN
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0xB4 180  
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0xB5 181  
  0x04,0x08,0x3E,0x60,0x3C,0x06,0x7C,0x00,  // 0xB6 182  (ś) ISO
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0xB7 183  
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0xB8 184  
  0x00,0x00,0x3C,0x06,0x3E,0x66,0x3E,0x04,  // 0xB9 185  (ą) WIN
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0xBA 186  
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0xBB 187  
  0x04,0x08,0x7E,0x0C,0x18,0x30,0x7E,0x00,  // 0xBC 188  (ź) ISO
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0xBD 189  
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0xBE 190  
  0x08,0x00,0x7E,0x0C,0x18,0x30,0x7E,0x00,  // 0xBF 191  (ż) ISO/WIN
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0xC0 192  
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0xC1 193  
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0xC2 194  
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0xC3 195  
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0xC4 196  
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0xC5 197  
  0x08,0x3C,0x66,0x60,0x60,0x66,0x3C,0x00,  // 0xC6 198  (Ć) ISO/WIN
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0xC7 199  
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0xC8 200  
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0xC9 201  
  0x7E,0x60,0x60,0x78,0x60,0x60,0x7E,0x04,  // 0xCA 202  (Ę) ISO/WIN
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0xCB 203  
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0xCC 204  
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0xCD 205  
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0xCE 206  
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0xCF 207  
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0xD0 208  
  0x08,0x66,0x76,0x7E,0x7E,0x6E,0x66,0x00,  // 0xD1 209  (Ń) ISO/WIN
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0xD2 210  
  0x08,0x3C,0x66,0x66,0x66,0x66,0x3C,0x00,  // 0xD3 211  (Ó) ISO/WIN
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0xD4 212  
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0xD5 213  
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0xD6 214  
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0xD7 215  
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0xD8 216  
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0xD9 217  
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0xDA 218  
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0xDB 219  
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0xDC 220  
  0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,  // 0xDD 221  
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0xDE 222  
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0xDF 223  
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0xE0 224  
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0xE1 225  
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0xE2 226  
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0xE3 227  
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0xE4 228  
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0xE5 229  
  0x04,0x08,0x3C,0x60,0x60,0x60,0x3C,0x00,  // 0xE6 230  (ć) ISO/WIN
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0xE7 231  
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0xE8 232  
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0xE9 233  
  0x00,0x00,0x3C,0x66,0x7E,0x60,0x3C,0x04,  // 0xEA 234  (ę) ISO/WIN
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0xEB 235  
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0xEC 236  
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0xED 237  
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0xEE 238  
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0xEF 239  
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0xF0 240  
  0x04,0x08,0x7C,0x66,0x66,0x66,0x66,0x00,  // 0xF1 241  (ń) ISO/WIN
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0xF2 242  
  0x04,0x08,0x3C,0x66,0x66,0x66,0x3C,0x00,  // 0xF3 243  (ó) ISO/WIN
  0xe0,0xa0,0xe0,0x1c,0x0b,0x0a,0x0b,0x00,  // 0xF4 244  deg TC
  0x00,0x80,0x80,0xac,0xca,0xaa,0xaa,0x00,  // 0xF5 245  kn
  0x00,0x80,0x80,0xaa,0xce,0xaa,0xaa,0x00,  // 0xF6 246  kmh
  0x00,0x00,0xd1,0xaa,0xab,0xa9,0xab,0x00,  // 0xF7 247  m/s
  0xbb,0x14,0x17,0x00,0x40,0x56,0x65,0x55,  // 0xF8 248  TC kn
  0xbb,0x14,0x17,0x00,0x40,0x55,0x67,0x55,  // 0xF9 249  TC kmh
  0xbb,0x14,0x17,0x00,0x53,0x76,0x51,0x57,  // 0xFA 250  TC m/s
  0x40,0x56,0x65,0x55,0x00,0xbb,0x14,0x17,  // 0xFB 251  kn TC
  0x40,0x55,0x67,0x55,0x00,0xbb,0x14,0x17,  // 0xFC 252  km/h TC
  0x53,0x76,0x51,0x57,0x00,0xbb,0x14,0x17,  // 0xFD 253  m/s TC
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0xFE 254  
  0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,  // 0xFF 255  
  // icons, add 0x100 (256)
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0x00   0  (empty)
  0x7E,0x81,0x3C,0x42,0x18,0x24,0x18,0x18,  // 0x01   1  (WiFi)
  0x7E,0xE7,0xE7,0xE7,0x81,0xC3,0xE7,0x7E,  // 0x02   2  (Download)
  0x18,0x5A,0x99,0x99,0x81,0x81,0x42,0x3C,  // 0x03   3  (Power)
  0x00,0x00,0xf0,0x89,0x69,0x19,0x19,0xf7,  // 0x04   4  Su -2y set, fi
  0x00,0x00,0xf8,0xae,0xa9,0x8f,0x89,0x89,  // 0x05   5  Ma
  0x00,0x00,0xfa,0x20,0x22,0x22,0x22,0x27,  // 0x06   6  Ti
  0x00,0x00,0x90,0xa7,0xc8,0xce,0xa8,0x9f,  // 0x07   7  Ke
  0x00,0x00,0xf8,0x26,0x29,0x29,0x29,0x26,  // 0x08   8  To
  0x00,0x00,0xe0,0x97,0x98,0xee,0x88,0x8f,  // 0x09   9  Pe
  0x00,0x00,0x80,0x8e,0x89,0x8f,0x89,0xe9,  // 0x0A  10  La
  0x00,0x00,0xf0,0x89,0x69,0x19,0x19,0xf7,  // 0x0B  11  Su -2y set, en
  0x00,0x00,0xf8,0xae,0xa9,0x89,0x89,0x8e,  // 0x0C  12  Mo
  0x00,0x00,0xf8,0x20,0x29,0x29,0x29,0x27,  // 0x0D  13  Tu
  0x00,0x00,0x88,0x8b,0x8c,0xaf,0xfc,0x53,  // 0x0E  14  We
  0x00,0x00,0xf8,0x25,0x25,0x27,0x25,0x25,  // 0x0F  15  Th
  0x00,0x00,0xf0,0x8e,0xe9,0x88,0x88,0x88,  // 0x10  16  Fr
  0x00,0x00,0xf0,0x86,0x69,0x1f,0x19,0xf9,  // 0x11  17  Sa
  0x00,0x70,0x80,0x89,0xe9,0x19,0x19,0xf7,  // 0x12  18  Su -1y set, fi
  0x00,0x88,0xf8,0xae,0x89,0x8f,0x89,0x89,  // 0x13  19  Ma
  0x00,0xf8,0x24,0x20,0x2c,0x24,0x24,0x2e,  // 0x14  20  Ti
  0x00,0x90,0x90,0xa7,0xc8,0xae,0x98,0x97,  // 0x15  21  Ke
  0x00,0xf8,0x20,0x26,0x29,0x29,0x29,0x26,  // 0x16  22  To
  0x00,0xe0,0x90,0x97,0xe8,0x8e,0x88,0x87,  // 0x17  23  Pe
  0x00,0x80,0x80,0x8e,0x89,0x8f,0x89,0xe9,  // 0x18  24  La	
  0x00,0x88,0xf8,0xae,0x89,0x89,0x89,0x86,  // 0x19  25  Su -1y set, en
  0x00,0xf8,0x20,0x29,0x29,0x29,0x29,0x27,  // 0x1A  26  Mo
  0x00,0x88,0x88,0x8b,0xac,0xaf,0xfc,0x57,  // 0x1B  27  Tu
  0x00,0xf8,0x20,0x29,0x29,0x2f,0x29,0x29,  // 0x1C  28  We
  0x00,0xf0,0x80,0x8e,0xe9,0x8e,0x89,0x89,  // 0x1D  29  Th
  0x00,0x70,0x80,0x8e,0xe9,0x1f,0x19,0xf9,  // 0x1E  30  Fr
  0x00,0x70,0x80,0x89,0xe9,0x19,0x19,0xf7,  // 0x1F  31  Sa
  0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,  // 0x20  32  64 dots on, linear
  0x7f,0xff,0xff,0xff,0xff,0xff,0xff,0xff,  // 0x21  33  63 dots on
  0x3f,0xff,0xff,0xff,0xff,0xff,0xff,0xff,  // 0x22  34  62 dots on
  0x1f,0xff,0xff,0xff,0xff,0xff,0xff,0xff,  // 0x23  35  61 dots on
  0x0f,0xff,0xff,0xff,0xff,0xff,0xff,0xff,  // 0x24  36  60 dots on
  0x07,0xff,0xff,0xff,0xff,0xff,0xff,0xff,  // 0x25  37  59 dots on
  0x03,0xff,0xff,0xff,0xff,0xff,0xff,0xff,  // 0x26  38  58 dots on
  0x01,0xff,0xff,0xff,0xff,0xff,0xff,0xff,  // 0x27  39  57 dots on
  0x00,0xff,0xff,0xff,0xff,0xff,0xff,0xff,  // 0x28  40  56 dots on
  0x00,0x7f,0xff,0xff,0xff,0xff,0xff,0xff,  // 0x29  41  55 dots on
  0x00,0x3f,0xff,0xff,0xff,0xff,0xff,0xff,  // 0x2A  42  54 dots on
  0x00,0x1f,0xff,0xff,0xff,0xff,0xff,0xff,  // 0x2B  43  53 dots on
  0x00,0x0f,0xff,0xff,0xff,0xff,0xff,0xff,  // 0x2C  44  52 dots on
  0x00,0x07,0xff,0xff,0xff,0xff,0xff,0xff,  // 0x2D  45  51 dots on
  0x00,0x03,0xff,0xff,0xff,0xff,0xff,0xff,  // 0x2E  46  50 dots on
  0x00,0x01,0xff,0xff,0xff,0xff,0xff,0xff,  // 0x2F  47  49 dots on
  0x00,0x00,0xff,0xff,0xff,0xff,0xff,0xff,  // 0x30  48  48 dots on
  0x00,0x00,0x7f,0xff,0xff,0xff,0xff,0xff,  // 0x31  49  47 dots on
  0x00,0x00,0x3f,0xff,0xff,0xff,0xff,0xff,  // 0x32  50  46 dots on
  0x00,0x00,0x1f,0xff,0xff,0xff,0xff,0xff,  // 0x33  51  45 dots on
  0x00,0x00,0x0f,0xff,0xff,0xff,0xff,0xff,  // 0x34  52  44 dots on
  0x00,0x00,0x07,0xff,0xff,0xff,0xff,0xff,  // 0x35  53  43 dots on
  0x00,0x00,0x03,0xff,0xff,0xff,0xff,0xff,  // 0x36  54  42 dots on
  0x00,0x00,0x01,0xff,0xff,0xff,0xff,0xff,  // 0x37  55  41 dots on
  0x00,0x00,0x00,0xff,0xff,0xff,0xff,0xff,  // 0x38  56  40 dots on
  0x00,0x00,0x00,0x7f,0xff,0xff,0xff,0xff,  // 0x39  57  39 dots on
  0x00,0x00,0x00,0x3f,0xff,0xff,0xff,0xff,  // 0x3A  58  38 dots on
  0x00,0x00,0x00,0x1f,0xff,0xff,0xff,0xff,  // 0x3B  59  37 dots on
  0x00,0x00,0x00,0x0f,0xff,0xff,0xff,0xff,  // 0x3C  50  36 dots on
  0x00,0x00,0x00,0x07,0xff,0xff,0xff,0xff,  // 0x3D  61  35 dots on
  0x00,0x00,0x00,0x03,0xff,0xff,0xff,0xff,  // 0x3E  62  34 dots on
  0x00,0x00,0x00,0x01,0xff,0xff,0xff,0xff,  // 0x3F  63  33 dots on
  0x00,0x00,0x00,0x00,0xff,0xff,0xff,0xff,  // 0x40  64  32 dots on
  0x00,0x00,0x00,0x00,0x7f,0xff,0xff,0xff,  // 0x41  65  31 dots on
  0x00,0x00,0x00,0x00,0x3f,0xff,0xff,0xff,  // 0x42  66  30 dots on
  0x00,0x00,0x00,0x00,0x1f,0xff,0xff,0xff,  // 0x43  67  29 dots on
  0x00,0x00,0x00,0x00,0x0f,0xff,0xff,0xff,  // 0x44  68  28 dots on
  0x00,0x00,0x00,0x00,0x07,0xff,0xff,0xff,  // 0x45  69  27 dots on
  0x00,0x00,0x00,0x00,0x03,0xff,0xff,0xff,  // 0x46  70  26 dots on
  0x00,0x00,0x00,0x00,0x01,0xff,0xff,0xff,  // 0x47  71  25 dots on
  0x00,0x00,0x00,0x00,0x00,0xff,0xff,0xff,  // 0x48  72  24 dots on
  0x00,0x00,0x00,0x00,0x00,0x7f,0xff,0xff,  // 0x49  73  23 dots on
  0x00,0x00,0x00,0x00,0x00,0x3f,0xff,0xff,  // 0x4A  74  22 dots on
  0x00,0x00,0x00,0x00,0x00,0x1f,0xff,0xff,  // 0x4B  75  21 dots on
  0x00,0x00,0x00,0x00,0x00,0x0f,0xff,0xff,  // 0x4C  76  20 dots on
  0x00,0x00,0x00,0x00,0x00,0x07,0xff,0xff,  // 0x4D  77  19 dots on
  0x00,0x00,0x00,0x00,0x00,0x03,0xff,0xff,  // 0x4E  78  18 dots on
  0x00,0x00,0x00,0x00,0x00,0x01,0xff,0xff,  // 0x4F  79  17 dots on
  0x00,0x00,0x00,0x00,0x00,0x00,0xff,0xff,  // 0x50  80  16 dots on
  0x00,0x00,0x00,0x00,0x00,0x00,0x7f,0xff,  // 0x51  81  15 dots on
  0x00,0x00,0x00,0x00,0x00,0x00,0x3f,0xff,  // 0x52  82  14 dots on
  0x00,0x00,0x00,0x00,0x00,0x00,0x1f,0xff,  // 0x53  83  13 dots on
  0x00,0x00,0x00,0x00,0x00,0x00,0x0f,0xff,  // 0x54  84  12 dots on
  0x00,0x00,0x00,0x00,0x00,0x00,0x07,0xff,  // 0x55  85  11 dots on
  0x00,0x00,0x00,0x00,0x00,0x00,0x03,0xff,  // 0x56  86  10 dots on
  0x00,0x00,0x00,0x00,0x00,0x00,0x01,0xff,  // 0x57  87   9 dots on
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xff,  // 0x58  88   8 dots on
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7f,  // 0x59  89   7 dots on
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3f,  // 0x5A  90   6 dots on
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1f,  // 0x5B  91   5 dots on
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0f,  // 0x5C  92   4 dots on
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x07,  // 0x5D  93   3 dots on
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,  // 0x5E  94   2 dots on
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,  // 0x5F  95   1 dot on
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0x60  96 all dots off
  0x7e,0xff,0xff,0xff,0xff,0xff,0xff,0x7e,  // 0x61  97  60 dots on, clock (corners blank)
  0x76,0xff,0xff,0xff,0xff,0xff,0xff,0x7e,  // 0x62  98  59 dots on
  0x76,0xf7,0xff,0xff,0xff,0xff,0xff,0x7e,  // 0x63  99  58 dots on 
  0x76,0xf7,0xf7,0xff,0xff,0xff,0xff,0x7e,  // 0x64 100  57 dots on 
  0x76,0xf7,0xf7,0xf7,0xff,0xff,0xff,0x7e,  // 0x65 101  56 dots on 
  0x72,0xf7,0xf7,0xf7,0xff,0xff,0xff,0x7e,  // 0x66 102  55 dots on 
  0x72,0xf3,0xf7,0xf7,0xff,0xff,0xff,0x7e,  // 0x67 103  54 dots on 
  0x70,0xf3,0xf7,0xf7,0xff,0xff,0xff,0x7e,  // 0x68 104  53 dots on 
  0x70,0xf3,0xf3,0xf7,0xff,0xff,0xff,0x7e,  // 0x69 105  52 dots on 
  0x70,0xf1,0xf3,0xf7,0xff,0xff,0xff,0x7e,  // 0x6A 106  51 dots on 
  0x70,0xf0,0xf3,0xf7,0xff,0xff,0xff,0x7e,  // 0x6B 107  50 dots on 
  0x70,0xf0,0xf1,0xf7,0xff,0xff,0xff,0x7e,  // 0x6C 108  49 dots on 
  0x70,0xf0,0xf0,0xf7,0xff,0xff,0xff,0x7e,  // 0x6D 109  48 dots on 
  0x70,0xf0,0xf0,0xf3,0xff,0xff,0xff,0x7e,  // 0x6E 110  47 dots on 
  0x70,0xf0,0xf0,0xf1,0xff,0xff,0xff,0x7e,  // 0x6F 111  46 dots on 
  0x70,0xf0,0xf0,0xf0,0xff,0xff,0xff,0x7e,  // 0x70 112  45 dots on 
  0x70,0xf0,0xf0,0xf0,0xfe,0xff,0xff,0x7e,  // 0x71 113  44 dots on 
  0x70,0xf0,0xf0,0xf0,0xfc,0xff,0xff,0x7e,  // 0x72 114  43 dots on 
  0x70,0xf0,0xf0,0xf0,0xf8,0xff,0xff,0x7e,  // 0x73 115  42 dots on 
  0x70,0xf0,0xf0,0xf0,0xf0,0xff,0xff,0x7e,  // 0x74 116  41 dots on 
  0x70,0xf0,0xf0,0xf0,0xf0,0xfe,0xff,0x7e,  // 0x75 117  40 dots on 
  0x70,0xf0,0xf0,0xf0,0xf0,0xfc,0xff,0x7e,  // 0x76 118  39 dots on 
  0x70,0xf0,0xf0,0xf0,0xf0,0xfc,0xfe,0x7e,  // 0x77 119  38 dots on 
  0x70,0xf0,0xf0,0xf0,0xf0,0xf8,0xfe,0x7e,  // 0x78 120  37 dots on 
  0x70,0xf0,0xf0,0xf0,0xf0,0xf8,0xfc,0x7e,  // 0x79 121  36 dots on 
  0x70,0xf0,0xf0,0xf0,0xf0,0xf8,0xfc,0x7c,  // 0x7A 122  35 dots on 
  0x70,0xf0,0xf0,0xf0,0xf0,0xf8,0xf8,0x7c,  // 0x7B 123  34 dots on 
  0x70,0xf0,0xf0,0xf0,0xf0,0xf8,0xf8,0x78,  // 0x7C 124  33 dots on 
  0x70,0xf0,0xf0,0xf0,0xf0,0xf0,0xf8,0x78,  // 0x7D 125  32 dots on 
  0x70,0xf0,0xf0,0xf0,0xf0,0xf0,0xf0,0x78,  // 0x7E 126  31 dots on 
  0x70,0xf0,0xf0,0xf0,0xf0,0xf0,0xf0,0x70,  // 0x7F 127  30 dots on 
  0x70,0xf0,0xf0,0xf0,0xf0,0xf0,0xf0,0x60,  // 0x81 128  29 dots on 
  0x70,0xf0,0xf0,0xf0,0xf0,0xf0,0xe0,0x60,  // 0x82 129  28 dots on 
  0x70,0xf0,0xf0,0xf0,0xf0,0xe0,0xe0,0x60,  // 0x83 130  27 dots on 
  0x70,0xf0,0xf0,0xf0,0xe0,0xe0,0xe0,0x60,  // 0x84 131  26 dots on 
  0x70,0xf0,0xf0,0xf0,0xe0,0xe0,0xe0,0x40,  // 0x85 132  25 dots on 
  0x70,0xf0,0xf0,0xf0,0xe0,0xe0,0xc0,0x40,  // 0x80 133  24 dots on 
  0x70,0xf0,0xf0,0xf0,0xe0,0xe0,0xc0,0x00,  // 0x86 134  23 dots on 
  0x70,0xf0,0xf0,0xf0,0xe0,0xc0,0xc0,0x00,  // 0x87 135  22 dots on 
  0x70,0xf0,0xf0,0xf0,0xe0,0xc0,0x80,0x00,  // 0x88 136  21 dots on 
  0x70,0xf0,0xf0,0xf0,0xe0,0xc0,0x00,0x00,  // 0x89 137  20 dots on 
  0x70,0xf0,0xf0,0xf0,0xe0,0x80,0x00,0x00,  // 0x8A 138  19 dots on 
  0x70,0xf0,0xf0,0xf0,0xe0,0x00,0x00,0x00,  // 0x8B 139  18 dots on 
  0x70,0xf0,0xf0,0xf0,0xc0,0x00,0x00,0x00,  // 0x8C 140  17 dots on 
  0x70,0xf0,0xf0,0xf0,0x80,0x00,0x00,0x00,  // 0x8D 141  16 dots on 
  0x70,0xf0,0xf0,0xf0,0x00,0x00,0x00,0x00,  // 0x8E 142  15 dots on 
  0x70,0xf0,0xf0,0x70,0x00,0x00,0x00,0x00,  // 0x8F 143  14 dots on 
  0x70,0xf0,0xf0,0x30,0x00,0x00,0x00,0x00,  // 0x90 144  13 dots on 
  0x70,0xf0,0xf0,0x10,0x00,0x00,0x00,0x00,  // 0x91 145  12 dots on 
  0x70,0xf0,0xf0,0x00,0x00,0x00,0x00,0x00,  // 0x92 146  11 dots on 
  0x70,0xf0,0x70,0x00,0x00,0x00,0x00,0x00,  // 0x93 147  10 dots on 
  0x70,0xf0,0x30,0x00,0x00,0x00,0x00,0x00,  // 0x94 148   9 dots on 
  0x70,0x70,0x30,0x00,0x00,0x00,0x00,0x00,  // 0x95 149   8 dots on 
  0x70,0x70,0x10,0x00,0x00,0x00,0x00,0x00,  // 0x96 150   7 dots on 
  0x70,0x30,0x10,0x00,0x00,0x00,0x00,0x00,  // 0x97 151   6 dots on 
  0x30,0x30,0x10,0x00,0x00,0x00,0x00,0x00,  // 0x98 152   5 dots on 
  0x30,0x10,0x10,0x00,0x00,0x00,0x00,0x00,  // 0x99 153   4 dots on 
  0x10,0x10,0x10,0x00,0x00,0x00,0x00,0x00,  // 0x9A 154   3 dots on 
  0x10,0x10,0x00,0x00,0x00,0x00,0x00,0x00,  // 0x9B 155   2 dots on 
  0x10,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // 0x9C 156   1 dot on
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00   // 0x9D 157 all dots off
};
#define ICONMAX 157

/* try https://xantorohara.github.io/led-matrix-editor/
 * and this bit of Perl to flip & split the output:
#!/usr/bin/perl
my @out = ();  my $offset = shift;
while (<>) {  next unless (/^\s*0x/);  chomp();  s{^\s*0x}{};  s{,.*}{};
  my $bytes = pack("H*", $_); my @chars = unpack("C*", $bytes);
  my @buf = ();  for (my $i = $#chars; $i > -1; $i--) {
    $p = ord pack 'B8', unpack 'b8', pack 'C', $chars[$i];
    push @buf, sprintf("0x%02x", $p);  }
  push @out, "  ".join(',',@buf).",  // ".sprintf("0x%03X %03d",$offset,$offset);
  $offset++;  }  print join("\n", @out) . "\n";
*/

// EOF
