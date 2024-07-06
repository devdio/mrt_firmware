#ifndef BBMATRIX_H
#define BBMATRIX_H

#include <Adafruit_NeoPixel.h>
#include "BbBoard.h"

const uint8_t characterA[5][5] = {
  {0, 1, 1, 1, 0},
  {0, 1, 0, 1, 0},
  {0, 1, 1, 1, 0},
  {0, 1, 0, 1, 0},
  {0, 1, 0, 1, 0}
};

const uint8_t characterB[5][5] = {
  {1, 1, 1, 1, 0},
  {1, 0, 0, 1, 0},
  {1, 1, 1, 1, 0},
  {1, 0, 0, 1, 0},
  {1, 1, 1, 1, 0}
};

const uint8_t characterC[5][5] = {
  {0, 1, 1, 1, 0},
  {1, 0, 0, 0, 0},
  {1, 0, 0, 0, 0},
  {1, 0, 0, 0, 0},
  {0, 1, 1, 1, 0}
};

const uint8_t characterD[5][5] = {
  {1, 1, 1, 0, 0},
  {1, 0, 0, 1, 0},
  {1, 0, 0, 1, 0},
  {1, 0, 0, 1, 0},
  {1, 1, 1, 0, 0}
};

const uint8_t characterE[5][5] = {
  {1, 1, 1, 1, 1},
  {1, 0, 0, 0, 0},
  {1, 1, 1, 0, 0},
  {1, 0, 0, 0, 0},
  {1, 1, 1, 1, 1}
};

const uint8_t characterF[5][5] = {
  {1, 1, 1, 1, 1},
  {1, 0, 0, 0, 0},
  {1, 1, 1, 0, 0},
  {1, 0, 0, 0, 0},
  {1, 0, 0, 0, 0}
};

const uint8_t characterG[5][5] = {
  {0, 1, 1, 1, 0},
  {1, 0, 0, 0, 0},
  {1, 0, 1, 1, 1},
  {1, 0, 0, 1, 0},
  {0, 1, 1, 1, 0}
};

const uint8_t characterH[5][5] = {
  {1, 0, 0, 0, 1},
  {1, 0, 0, 0, 1},
  {1, 1, 1, 1, 1},
  {1, 0, 0, 0, 1},
  {1, 0, 0, 0, 1}
};

const uint8_t characterI[5][5] = {
  {0, 1, 1, 1, 0},
  {0, 0, 1, 0, 0},
  {0, 0, 1, 0, 0},
  {0, 0, 1, 0, 0},
  {0, 1, 1, 1, 0}
};

const uint8_t characterJ[5][5] = {
  {0, 1, 1, 1, 1},
  {0, 0, 0, 1, 0},
  {0, 0, 0, 1, 0},
  {1, 0, 0, 1, 0},
  {0, 1, 1, 0, 0}
};

const uint8_t characterK[5][5] = {
  {1, 0, 0, 1, 0},
  {1, 0, 1, 0, 0},
  {1, 1, 0, 0, 0},
  {1, 0, 1, 0, 0},
  {1, 0, 0, 1, 0}
};

const uint8_t characterL[5][5] = {
  {1, 0, 0, 0, 0},
  {1, 0, 0, 0, 0},
  {1, 0, 0, 0, 0},
  {1, 0, 0, 0, 0},
  {1, 1, 1, 1, 1}
};

const uint8_t characterM[5][5] = {
  {1, 0, 0, 0, 1},
  {1, 1, 0, 1, 1},
  {1, 0, 1, 0, 1},
  {1, 0, 0, 0, 1},
  {1, 0, 0, 0, 1}
};

const uint8_t characterN[5][5] = {
  {1, 0, 0, 0, 1},
  {1, 1, 0, 0, 1},
  {1, 0, 1, 0, 1},
  {1, 0, 0, 1, 1},
  {1, 0, 0, 0, 1}
};

const uint8_t characterO[5][5] = {
  {0, 1, 1, 1, 0},
  {1, 0, 0, 0, 1},
  {1, 0, 0, 0, 1},
  {1, 0, 0, 0, 1},
  {0, 1, 1, 1, 0}
};

const uint8_t characterP[5][5] = {
  {1, 1, 1, 1, 0},
  {1, 0, 0, 0, 1},
  {1, 1, 1, 1, 0},
  {1, 0, 0, 0, 0},
  {1, 0, 0, 0, 0}
};

const uint8_t characterQ[5][5] = {
  {0, 1, 1, 1, 0},
  {1, 0, 0, 0, 1},
  {1, 0, 0, 0, 1},
  {1, 0, 1, 1, 0},
  {0, 1, 1, 0, 1}
};

const uint8_t characterR[5][5] = {
  {1, 1, 1, 1, 0},
  {1, 0, 0, 0, 1},
  {1, 1, 1, 1, 0},
  {1, 0, 1, 0, 0},
  {1, 0, 0, 1, 0}
};

const uint8_t characterS[5][5] = {
  {0, 1, 1, 1, 1},
  {1, 0, 0, 0, 0},
  {0, 1, 1, 1, 0},
  {0, 0, 0, 0, 1},
  {1, 1, 1, 1, 0}
};

const uint8_t characterT[5][5] = {
  {1, 1, 1, 1, 1},
  {0, 0, 1, 0, 0},
  {0, 0, 1, 0, 0},
  {0, 0, 1, 0, 0},
  {0, 0, 1, 0, 0}
};

const uint8_t characterU[5][5] = {
  {1, 0, 0, 0, 1},
  {1, 0, 0, 0, 1},
  {1, 0, 0, 0, 1},
  {1, 0, 0, 0, 1},
  {0, 1, 1, 1, 0}
};

const uint8_t characterV[5][5] = {
  {1, 0, 0, 0, 1},
  {1, 0, 0, 0, 1},
  {0, 1, 0, 1, 0},
  {0, 1, 0, 1, 0},
  {0, 0, 1, 0, 0}
};

const uint8_t characterW[5][5] = {
  {1, 0, 0, 0, 1},
  {1, 0, 0, 0, 1},
  {1, 0, 1, 0, 1},
  {1, 1, 0, 1, 1},
  {0, 1, 0, 1, 0}
};

const uint8_t characterX[5][5] = {
  {1, 0, 0, 0, 1},
  {0, 1, 0, 1, 0},
  {0, 0, 1, 0, 0},
  {0, 1, 0, 1, 0},
  {1, 0, 0, 0, 1}
};

const uint8_t characterY[5][5] = {
  {1, 0, 0, 0, 1},
  {0, 1, 0, 1, 0},
  {0, 0, 1, 0, 0},
  {0, 0, 1, 0, 0},
  {0, 0, 1, 0, 0}
};

const uint8_t characterZ[5][5] = {
  {1, 1, 1, 1, 1},
  {0, 0, 0, 1, 0},
  {0, 0, 1, 0, 0},
  {0, 1, 0, 0, 0},
  {1, 1, 1, 1, 1}
};

// 다른 문자를 추가하세요...
// 숫자 0
const uint8_t character0[5][5] = {
  {0, 1, 1, 1, 0},
  {1, 0, 0, 0, 1},
  {1, 0, 0, 1, 1},
  {1, 0, 1, 0, 1},
  {0, 1, 1, 1, 0}
};

// 숫자 1
const uint8_t character1[5][5] = {
  {0, 0, 1, 0, 0},
  {0, 1, 1, 0, 0},
  {1, 0, 1, 0, 0},
  {0, 0, 1, 0, 0},
  {1, 1, 1, 1, 1}
};

// 숫자 2
const uint8_t character2[5][5] = {
  {0, 1, 1, 1, 0},
  {1, 0, 0, 0, 1},
  {0, 0, 1, 0, 0},
  {0, 1, 0, 0, 0},
  {1, 1, 1, 1, 1}
};

// 숫자 3
const uint8_t character3[5][5] = {
  {1, 1, 1, 1, 0},
  {0, 0, 0, 0, 1},
  {0, 1, 1, 1, 0},
  {0, 0, 0, 0, 1},
  {1, 1, 1, 1, 0}
};

// 숫자 4
const uint8_t character4[5][5] = {
  {1, 0, 0, 0, 1},
  {1, 0, 0, 0, 1},
  {1, 1, 1, 1, 1},
  {0, 0, 0, 0, 1},
  {0, 0, 0, 0, 1}
};

// 숫자 5
const uint8_t character5[5][5] = {
  {1, 1, 1, 1, 1},
  {1, 0, 0, 0, 0},
  {1, 1, 1, 1, 0},
  {0, 0, 0, 0, 1},
  {1, 1, 1, 1, 0}
};

// 숫자 6
const uint8_t character6[5][5] = {
  {0, 1, 1, 1, 0},
  {1, 0, 0, 0, 0},
  {1, 1, 1, 1, 0},
  {1, 0, 0, 0, 1},
  {0, 1, 1, 1, 0}
};

// 숫자 7
const uint8_t character7[5][5] = {
  {1, 1, 1, 1, 1},
  {0, 0, 0, 0, 1},
  {0, 0, 0, 1, 0},
  {0, 0, 1, 0, 0},
  {0, 1, 0, 0, 0}
};

// 숫자 8
const uint8_t character8[5][5] = {
  {0, 1, 1, 1, 0},
  {1, 0, 0, 0, 1},
  {0, 1, 1, 1, 0},
  {1, 0, 0, 0, 1},
  {0, 1, 1, 1, 0}
};

// 숫자 9
const uint8_t character9[5][5] = {
  {0, 1, 1, 1, 0},
  {1, 0, 0, 0, 1},
  {0, 1, 1, 1, 1},
  {0, 0, 0, 0, 1},
  {0, 1, 1, 1, 0}
};


void BbMatrix_displayColor(Adafruit_NeoPixel* _pixels, RGB color) {
  // for (int row = 0; row < 5; row++) {
  //   for (int col = 0; col < 5; col++) {
  //     int pixelIndex = col * 5 + row;
  //     _pixels->setPixelColor(pixelIndex, _pixels->Color(color.r, color.g, color.b));
  //   }
  // }
  _pixels->fill(_pixels->Color(color.r, color.g, color.b), 0, 25);
  _pixels->show();
}


void BbMatrix_displaySymbol(Adafruit_NeoPixel* _pixels, RGB symbol[5][5]) {
  for (int row = 0; row < 5; row++) {
    for (int col = 0; col < 5; col++) {
      int pixelIndex = col * 5 + row;
      RGB color = symbol[row][col];
      _pixels->setPixelColor(pixelIndex, _pixels->Color(color.r, color.g, color.b));
    }
  }
  _pixels->show();
}

void BbMatrix_displaySymbol(Adafruit_NeoPixel* _pixels, const uint8_t symbol[5][5], RGB color) {
  for (int row = 0; row < 5; row++) {
    for (int col = 0; col < 5; col++) {
      int pixelIndex = col * 5 + row;
      int flag = symbol[row][col];
      if (flag != 0) {
        _pixels->setPixelColor(pixelIndex, _pixels->Color(color.r, color.g, color.b));
      } else {
        _pixels->setPixelColor(pixelIndex, _pixels->Color(0, 0, 0));
      }
    }
  }
  _pixels->show();
}

void BbMatrix_writeLine(Adafruit_NeoPixel* _pixels, int i, char data, RGB color) {

  uint8_t line = (uint8_t)data;

  if (line & 0b00010000) {
    _pixels->setPixelColor(0+i, _pixels->Color(color.r, color.g, color.b));
  } else {
    _pixels->setPixelColor(0+i, _pixels->Color(0, 0, 0));
  }
  
  if (line & 0b00001000) {
    _pixels->setPixelColor(5+i, _pixels->Color(color.r, color.g, color.b));
  } else {
    _pixels->setPixelColor(5+i, _pixels->Color(0, 0, 0));
  }

  if (line & 0b00000100) {
    _pixels->setPixelColor(10+i, _pixels->Color(color.r, color.g, color.b));
  } else {
    _pixels->setPixelColor(10+i, _pixels->Color(0, 0, 0));
  }

  if (line & 0b00000010) {
    _pixels->setPixelColor(15+i, _pixels->Color(color.r, color.g, color.b));
  } else {
    _pixels->setPixelColor(15+i, _pixels->Color(0, 0, 0));
  }

  if (line & 0b00000001) {
    _pixels->setPixelColor(20+i, _pixels->Color(color.r, color.g, color.b));
  } else {
    _pixels->setPixelColor(20+i, _pixels->Color(0, 0, 0));
  }
}

void BbMatrix_display(Adafruit_NeoPixel* _pixels, char* data, RGB color) {
  _pixels->clear();
  BbMatrix_writeLine(_pixels, 0, data[0], color);
  BbMatrix_writeLine(_pixels, 1, data[1], color);
  BbMatrix_writeLine(_pixels, 2, data[2], color);
  BbMatrix_writeLine(_pixels, 3, data[3], color);
  BbMatrix_writeLine(_pixels,4, data[4], color);
  _pixels->show();
}

void BbMatrix_display(Adafruit_NeoPixel* _pixels, uint8_t i, uint8_t j) {
  uint8_t pos = i*5 + j;

  _pixels->setPixelColor(pos, _pixels->Color(10, 10, 0));
  _pixels->show();
}

void BbMatrix_displayPixel(Adafruit_NeoPixel* _pixels, uint8_t i, uint8_t j, RGB color) {
  uint8_t pos = i*5 + j;

  _pixels->setPixelColor(pos, _pixels->Color(color.r, color.g, color.b));
  _pixels->show();
}

void BbMatrix_displayPixel(Adafruit_NeoPixel* _pixels, uint8_t num, RGB color) {
  
  _pixels->setPixelColor(num, _pixels->Color(color.r, color.g, color.b));
  _pixels->show();
}


// void BbMatrix_displayColor(RGB color) {
//   // for (int row = 0; row < 5; row++) {
//   //   for (int col = 0; col < 5; col++) {
//   //     int pixelIndex = col * 5 + row;
//   //     _pixels->setPixelColor(pixelIndex, _pixels->Color(color.r, color.g, color.b));
//   //   }
//   // }
//   // _pixels->show();
// }

void BbMatrix_displayNum(Adafruit_NeoPixel* _pixels, char c, RGB color) {
    switch(c)
    {
        case 0:
          BbMatrix_displaySymbol(_pixels, character0, color); break;
        case 1:
          BbMatrix_displaySymbol(_pixels, character1, color); break;
        case 2:
          BbMatrix_displaySymbol(_pixels, character2, color); break;
        case 3:
          BbMatrix_displaySymbol(_pixels, character3, color); break;
        case 4:
          BbMatrix_displaySymbol(_pixels, character4, color); break;
        case 5:
          BbMatrix_displaySymbol(_pixels, character5, color); break;
        case 6:
          BbMatrix_displaySymbol(_pixels, character6, color); break;
        case 7:
          BbMatrix_displaySymbol(_pixels, character7, color); break;
        case 8:
          BbMatrix_displaySymbol(_pixels, character8, color); break;
        case 9:
          BbMatrix_displaySymbol(_pixels, character9, color); break;
        default:
          BbMatrix_displaySymbol(_pixels, characterX, color); break;
  };
  _pixels->show();
}


void BbMatrix_displayChar(Adafruit_NeoPixel* _pixels, char c, RGB color) {
    
    switch(c)
    {
        case 'A':
          BbMatrix_displaySymbol(_pixels, characterA, color); break;
        case 'B':
          BbMatrix_displaySymbol(_pixels, characterB, color); break;
        case 'C':
          BbMatrix_displaySymbol(_pixels, characterC, color); break;
        case 'D':
          BbMatrix_displaySymbol(_pixels, characterD, color); break;
        case 'E':
          BbMatrix_displaySymbol(_pixels, characterE, color); break;
        case 'F':
          BbMatrix_displaySymbol(_pixels, characterF, color); break;
        case 'G':
          BbMatrix_displaySymbol(_pixels, characterG, color); break;
        case 'H':
          BbMatrix_displaySymbol(_pixels, characterH, color); break;
        case 'I':
          BbMatrix_displaySymbol(_pixels, characterI, color); break;
        case 'J':
          BbMatrix_displaySymbol(_pixels, characterJ, color); break;
        case 'K':
          BbMatrix_displaySymbol(_pixels, characterK, color); break;
        case 'L':
          BbMatrix_displaySymbol(_pixels, characterL, color); break;
        case 'M':
          BbMatrix_displaySymbol(_pixels, characterM, color); break;
        case 'N':
          BbMatrix_displaySymbol(_pixels, characterN, color); break;
        case 'O':
          BbMatrix_displaySymbol(_pixels, characterO, color); break;
        case 'P':
          BbMatrix_displaySymbol(_pixels, characterP, color); break;
        case 'Q':
          BbMatrix_displaySymbol(_pixels, characterQ, color); break;
        case 'R':
          BbMatrix_displaySymbol(_pixels, characterR, color); break;
        case 'S':
          BbMatrix_displaySymbol(_pixels, characterS, color); break;
        case 'T':
          BbMatrix_displaySymbol(_pixels, characterT, color); break;
        case 'U':
          BbMatrix_displaySymbol(_pixels, characterU, color); break;
        case 'V':
          BbMatrix_displaySymbol(_pixels, characterV, color); break;
        case 'W':
          BbMatrix_displaySymbol(_pixels, characterW, color); break;
        case 'X':
          BbMatrix_displaySymbol(_pixels, characterX, color); break;
        case 'Y':
          BbMatrix_displaySymbol(_pixels, characterY, color); break;
        case 'Z':
          BbMatrix_displaySymbol(_pixels, characterZ, color); break;
        default:
          BbMatrix_displaySymbol(_pixels, characterX, color); break;
  };
  _pixels->show();
}

void BbMatrix_rainbow(Adafruit_NeoPixel* _pixels, int idx, int wait) {
  // idx 효과 번호 
  for(long firstPixelHue = 0; firstPixelHue < 5*65536; firstPixelHue += 256) {
    _pixels->rainbow(firstPixelHue);
    _pixels->show();
    delay(1);
  }
}

void BbMatrix_theaterChaseRainbow(Adafruit_NeoPixel* _pixels, int idx, int wait) {
  int firstPixelHue = 0;     // First pixel starts at red (hue 0)
  for(int a=0; a<15; a++) {  // Repeat 15 times...
    for(int b=0; b<3; b++) { //  'b' counts from 0 to 2...
      _pixels->clear();         //   Set all pixels in RAM to 0 (off)
      for(int c=b; c<_pixels->numPixels(); c += 3) {
        int      hue   = firstPixelHue + c * 65536L / _pixels->numPixels();
        uint32_t color = _pixels->gamma32(_pixels->ColorHSV(hue)); // hue -> RGB
        _pixels->setPixelColor(c, color); // Set pixel 'c' to value 'color'
      }
      _pixels->show();                // Update strip with new contents
      delay(50);                 // Pause for a moment
      firstPixelHue += 65536 / 90; // One cycle of color wheel over 90 frames
    }
  }
}


void BbMatrix_colorWipe(Adafruit_NeoPixel* _pixels, RGB color, int wait) {
  for(int i=0; i<_pixels->numPixels(); i++) { // For each pixel in strip...
    _pixels->setPixelColor(i, _pixels->Color(color.r, color.g, color.b)); 
    _pixels->show(); //  Update strip to match
    delay(wait);  //  Pause for a moment
  }
}

void BbMatrix_effect(Adafruit_NeoPixel* _pixels, int idx, int wait) {
  if (idx == 0) {
    BbMatrix_rainbow(_pixels, idx, wait);
    BbMatrix_displayColor(_pixels, RGB(0, 0, 0));
  } else if(idx == 1) {
    BbMatrix_theaterChaseRainbow(_pixels, idx, wait);
    BbMatrix_displayColor(_pixels, RGB(0, 0, 0));
  } else if(idx == 2) {
    BbMatrix_colorWipe(_pixels, RGB(255,   0,   0), 50);
    BbMatrix_colorWipe(_pixels, RGB(  0, 255,   0), 50);
    BbMatrix_colorWipe(_pixels, RGB(  0,   0, 255), 50);
    BbMatrix_displayColor(_pixels, RGB(0, 0, 0));
  } else {
    BbMatrix_rainbow(_pixels, idx, wait);
    BbMatrix_displayColor(_pixels, RGB(0, 0, 0));
  }
}

void BbMatrix_setBrightness(Adafruit_NeoPixel* _pixels, int value) {
  _pixels->setBrightness(value);
}

void BbMatrix_clear(Adafruit_NeoPixel* _pixels) {
  _pixels->clear();
}

void BbMatrix_displayRow(Adafruit_NeoPixel* _pixels, uint8_t row, char* data, RGB color) {
  BbMatrix_writeLine(_pixels, row, data[0], color);
  _pixels->show();
}
#endif