/*
 * Helical tire utility
 *
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



// create a helical extrusion by copying the entire model to discrete layers
// the source model must be a single layer height
// gcc -g -O2 -o helix helix.c -lm
// ./helix <input> <output>
// ./helix tire.stl tire2.stl


#include "3dstuff.h"

//#define TOTAL_H 31.0 // parallel tread
#define TOTAL_H 60.0 // V tread
// rotation in degrees (360/columns)
#define TOTAL_ANGLE (360.0 / 18)
// make a V tread
#define DO_V 1

// void selectiveRotate(vector *coord)
// {
//     if(coord->z >= Z_THRESHOLD)
//     {
//         vector polar = XYZToPolar(*coord);
//         polar.x += angle;
//         *coord = polarToXYZ(polar);
//     }
// }

int main(int argc, char *argv[])
{
    if(argc < 3)
    {
        printf("Usage: helix <input> <output>\n");
        return 1;
    }
    
    char *inpath = argv[1];
    char *outpath = argv[2];

    int count;
    triangle_t *triangles = read_stl(inpath, &count);
    if(!triangles)
    {
        return 1;
    }

// get layer height from model
    int i, j;
    double min_z = 65535;
    double max_z = -65535;
    for(i = 0; i < count; i++)
    {
        triangle_t *triangle = &triangles[i];
        for(j = 0; j < 3; j++)
        {
            if(triangle->coords[j].z < min_z)
            {
                min_z = triangle->coords[j].z;
            }
            else
            if(triangle->coords[j].z > max_z)
            {
                max_z = triangle->coords[j].z;
            }
        }
    }
    
    double layer_h = max_z - min_z;
    printf("layer_h=%f\n", layer_h);


#ifdef DO_Z
    
#endif

    int total_layers = (int)(TOTAL_H / layer_h);
    printf("total_layers=%d\n", total_layers);

    open_stl(outpath);
    int layer;
    for(layer = 0; layer < total_layers; layer++)
    {
        double angle;
#ifdef DO_V
        if(layer <= total_layers / 2)
        {
            angle = toRad(TOTAL_ANGLE * layer / (total_layers / 2));
        }
        else
        {
            angle = toRad(TOTAL_ANGLE * (total_layers - layer) / (total_layers / 2));
        }
#else // DO_V
        angle = toRad(TOTAL_ANGLE * layer / total_layers);
#endif // !DO_V

        double z = layer * layer_h;
        for(i = 0; i < count; i++)
        {
            triangle_t *src = &triangles[i];
// copy the triangle
            triangle_t dst = *src;

            for(j = 0; j < 3; j++)
            {
// offset Z
                dst.coords[j].z += z;
// rotate coord
                vector polar = XYZToPolar(dst.coords[j]);
                polar.x += angle;
                dst.coords[j] = polarToXYZ(polar);
            }

// write it
            writeTriangle(dst.coords[0], dst.coords[1], dst.coords[2]);
        }
    }

    close_stl();
}


















