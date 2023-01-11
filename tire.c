/*
 * 3D printed truck
 * Copyright (C) 2021 Adam Williams <broadcast at earthling dot net>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 * 
 */



// synthesize a tire.  unfinished
// gcc -g -O2 -o tire tire.c -lm


#include "3dstuff.h"


void make_tire()
{
// inner diameter
    float iDiameter = 46;
// inner shell thickness
    float iThickness = 1.2;
// outer shell thickness
    float oThickness = 1.2;
// radius offset of center slice of tread
    float bulge = 5;
// arc angle defining the bulge
    float bulgeArc = toRad(90);
// outer diameter + tread + bulge
    float oDiameter = 110;
    float treadThickness = 3;
    float treadW = 8;
// tread subdivisions & gap subdivisions
    float treadSubdivide = 3;
// offset of the center of the tread V
    float treadOffset = 20;
    int numTreads = 18;
// layers of hexagons in a column
    int layers = 3;
// number of hexagon columns in a layer.  Each column is 2 hexagons side by side
    int columns = 24;
// thickness of hexagon wall
    float wallThickness = .8;
// tire width
    float w = 59.6;
// slices along width.  Only 2 required when using a loft
    float slices = 2;


    int slice;
    float minFraction = cos(bulgeArc);
    for(slice = 0; slice < slices + 1; slice++)
    {
        float z = w * slice / slices;
        float sliceFraction = 0.0;
        if(slice >= slices / 2)
        {
            sliceFraction = (cos(bulgeArc * (slice - slices / 2) / (slices / 2)) - minFraction) /
                (1.0 - minFraction);
        }
        else
        {
            sliceFraction = (cos(bulgeArc * (slices / 2 - slice) / (slices / 2)) - minFraction) /
                (1.0 - minFraction);
//printf("make_tire %d cos=%f\n",
//__LINE__,
//cos(bulgeArc * (slices / 2 - slice) / (slices / 2)));
        }
        float oDiameter2 = oDiameter - (bulge - sliceFraction * bulge) * 2;

printf("make_tire %d oDiameter2=%f\n", __LINE__, oDiameter2);
    }








}


void main(int argc, char *argv[])
{
    open_stl("tire.stl");

    make_tire();
    
    close_stl();
}











