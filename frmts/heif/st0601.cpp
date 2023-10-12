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
 
#include "st0601.h"
#include <cpl_port.h>

Abstract0601Parser::~Abstract0601Parser()
{
}

ST0601::ST0601()
{
    registry[65] = new ST0601Version();
    registry[82] = new CornerLatitudePoint1();
    registry[83] = new CornerLongitudePoint1();
    registry[84] = new CornerLatitudePoint2();
    registry[85] = new CornerLongitudePoint2();
    registry[86] = new CornerLatitudePoint3();
    registry[87] = new CornerLongitudePoint3();
    registry[88] = new CornerLatitudePoint4();
    registry[89] = new CornerLongitudePoint4();
}

ST0601::~ST0601()
{
}

const char * ST0601::lookupTagName(int tag)
{
    if (registry.count(tag))
    {
        Abstract0601Parser *parser = registry[tag];
        return parser->getTagName();
    }
    switch(tag)
    {
        case 1:
            return "Checksum";
        default:
            return "TODO tag";
    }
}

const char * ST0601::decodeValue(int tag, std::vector<GByte> data, size_t *dataOffset)
{
    // TODO: get BER length instead
    int length = data[*dataOffset];
    *dataOffset += 1;
    if (registry.count(tag))
    {
        Abstract0601Parser *parser = registry[tag];
        std::vector<GByte> v(data.begin() + *dataOffset, data.begin() + (*dataOffset) + length);
        /*
        for (size_t j = 0; j < v.size(); j++)
        {
            printf("%li: 0x%02x, ", j, v[j]);
        }
        printf("\n");
        */
        *dataOffset += length;
        return parser->decodeValue(v);
    }
    *dataOffset += length;
    return "TODO value";
}


