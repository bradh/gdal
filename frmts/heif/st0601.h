/******************************************************************************
 * Copyright (c) 2023, Brad Hards
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 ******************************************************************************/

#ifndef GDAL_HEIF_ST0601_H_INCLUDED
#define GDAL_HEIF_ST0601_H_INCLUDED

#include <cpl_port.h>
#include <cstddef>
#include <map>
#include <string>
#include <vector>

class Abstract0601Parser
{
    public:
        virtual ~Abstract0601Parser();
        virtual const char* getTagName() const = 0;
        virtual const char* decodeValue(std::vector<GByte>) const = 0;

    private:
};

class ST0601Version : public Abstract0601Parser
{
    public:
        const char* getTagName() const override
        {
            return "ST 0601 Version";
        }

        const char* decodeValue(std::vector<GByte> v) const override
        {
            char* formattedValue;
            asprintf(&formattedValue, "ST 0601.%i", v[0]);
            return formattedValue;
        }

        ~ST0601Version()
        {
        }
};

class CornerLatitudePoint : public Abstract0601Parser
{
    public:
        double getValue(std::vector<GByte> v) const
        {
            int klv = (v[0] << 24) + (v[1] << 16) + (v[2] << 8) + v[3];
            double softVal = klv * 180.0 / 4294967294.0;
            return softVal;
        }

        const char* decodeValue(std::vector<GByte> v) const override
        {
            char* formattedValue;
            asprintf(&formattedValue, "%f", getValue(v));
            return formattedValue;
        }
};

class CornerLatitudePoint1 : public CornerLatitudePoint
{
    public:
        const char* getTagName() const override
        {
            return "Corner Latitude Point 1 (Full)";
        }

        ~CornerLatitudePoint1() {}
};

class CornerLatitudePoint2 : public CornerLatitudePoint
{
    public:
        const char* getTagName() const override
        {
            return "Corner Latitude Point 2 (Full)";
        }

        ~CornerLatitudePoint2() {}
};

class CornerLatitudePoint3 : public CornerLatitudePoint
{
    public:
        const char* getTagName() const override
        {
            return "Corner Latitude Point 3 (Full)";
        }

        ~CornerLatitudePoint3() {}
};

class CornerLatitudePoint4 : public CornerLatitudePoint
{
    public:
        const char* getTagName() const override
        {
            return "Corner Latitude Point 4 (Full)";
        }

        ~CornerLatitudePoint4() {}
};


class CornerLongitudePoint : public Abstract0601Parser
{
    public:
        double getValue(std::vector<GByte> v) const
        {
            int klv = (v[0] << 24) + (v[1] << 16) + (v[2] << 8) + v[3];
            double softVal = klv * 360.0 / 4294967294.0;
            return softVal;
        }

        const char* decodeValue(std::vector<GByte> v) const override
        {
            char* formattedValue;
            asprintf(&formattedValue, "%f", getValue(v));
            return formattedValue;
        }
};

class CornerLongitudePoint1 : public CornerLongitudePoint
{
    public:
        const char* getTagName() const override
        {
            return "Corner Longitude Point 1 (Full)";
        }

        ~CornerLongitudePoint1() {}
};

class CornerLongitudePoint2 : public CornerLongitudePoint
{
    public:
        const char* getTagName() const override
        {
            return "Corner Longitude Point 2 (Full)";
        }

        ~CornerLongitudePoint2() {}
};

class CornerLongitudePoint3 : public CornerLongitudePoint
{
    public:
        const char* getTagName() const override
        {
            return "Corner Longitude Point 3 (Full)";
        }

        ~CornerLongitudePoint3() {}
};

class CornerLongitudePoint4 : public CornerLongitudePoint
{
    public:
        const char* getTagName() const override
        {
            return "Corner Longitude Point 4 (Full)";
        }

        ~CornerLongitudePoint4() {}
};

class ST0601
{
    public:
        ST0601();
        virtual ~ST0601();

        const char * lookupTagName(int tag);
        const char * decodeValue(int tag, std::vector<GByte> data, size_t *dataOffset);

    private:
        std::map<int, Abstract0601Parser*> registry;
};

#endif
