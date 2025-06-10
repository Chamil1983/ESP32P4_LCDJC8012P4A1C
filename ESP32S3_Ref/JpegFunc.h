/*******************************************************************************
 * JPEGDEC related function - Waveshare ESP32-S3-Touch-LCD-5 compatible version
 *
 * Dependent libraries:
 * JPEGDEC: https://github.com/bitbank2/JPEGDEC.git
 ******************************************************************************/
#ifndef _JPEGFUNC_H_
#define _JPEGFUNC_H_

#include <JPEGDEC.h>
#include <SD.h>

static JPEGDEC _jpeg;
static File _f;
static int _x, _y, _x_bound, _y_bound;

static void *jpegOpenFile(const char *szFilename, int32_t *pFileSize)
{
  Serial.println("jpegOpenFile");
  _f = SD.open(szFilename, "r");
  if (!_f) {
    Serial.println("Failed to open file");
    return NULL;
  }

  *pFileSize = _f.size();
  return &_f;
}

static void jpegCloseFile(void *pHandle)
{
    Serial.println("jpegCloseFile");
    File *f = static_cast<File *>(pHandle);
    if (f != NULL && *f) {
        f->close();
    }
}

static int32_t jpegReadFile(JPEGFILE *pFile, uint8_t *pBuf, int32_t iLen)
{
    Serial.printf("jpegReadFile, iLen: %d\n", iLen);
    File *f = static_cast<File *>(pFile->fHandle);
    if (f == NULL || !(*f)) {
        return 0;
    }
    size_t r = f->read(pBuf, iLen);
    return r;
}

static int32_t jpegSeekFile(JPEGFILE *pFile, int32_t iPosition)
{
    Serial.printf("jpegSeekFile, pFile->iPos: %d, iPosition: %d\n", pFile->iPos, iPosition);
    File *f = static_cast<File *>(pFile->fHandle);
    if (f == NULL || !(*f)) {
        return 0;
    }
    f->seek(iPosition);
    return iPosition;
}

static void jpegDraw(
    const char *filename, JPEG_DRAW_CALLBACK *jpegDrawCallback, bool useBigEndian,
    int x, int y, int widthLimit, int heightLimit)
{
    _x = x;
    _y = y;
    _x_bound = _x + widthLimit - 1;
    _y_bound = _y + heightLimit - 1;

    Serial.printf("jpegDraw: %s, pos: %d,%d, limits: %d x %d\n", 
                 filename, x, y, widthLimit, heightLimit);

    if (!_jpeg.open(filename, jpegOpenFile, jpegCloseFile, jpegReadFile, jpegSeekFile, jpegDrawCallback)) {
        Serial.println("Error opening JPEG file");
        return;
    }

    // scale to fit height
    int _scale;
    int iMaxMCUs;
    float ratio = (float)_jpeg.getHeight() / heightLimit;
    
    Serial.printf("JPEG dimensions: %d x %d, ratio: %.2f\n", 
                 _jpeg.getWidth(), _jpeg.getHeight(), ratio);
                 
    if (ratio <= 1)
    {
        _scale = 0; // JPEG_SCALE_NONE
        iMaxMCUs = widthLimit / 16;
    }
    else if (ratio <= 2)
    {
        _scale = JPEG_SCALE_HALF;
        iMaxMCUs = widthLimit / 8;
    }
    else if (ratio <= 4)
    {
        _scale = JPEG_SCALE_QUARTER;
        iMaxMCUs = widthLimit / 4;
    }
    else
    {
        _scale = JPEG_SCALE_EIGHTH;
        iMaxMCUs = widthLimit / 2;
    }
    
    Serial.printf("Using scale: %d, maxMCUs: %d\n", _scale, iMaxMCUs);
    
    _jpeg.setMaxOutputSize(iMaxMCUs);
    if (useBigEndian)
    {
        _jpeg.setPixelType(RGB565_BIG_ENDIAN);
    }
    
    if (_jpeg.decode(x, y, _scale) != 1) {
        Serial.println("Error decoding JPEG");
    } else {
        Serial.println("JPEG decoded successfully");
    }
    
    _jpeg.close();
}

#endif // _JPEGFUNC_H_