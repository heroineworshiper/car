// create motor tables from the TEST_MOTORS output
// paste the MOTORS lines into stdin followed by ctrl-d to process

// gcc -o motor_table motor_table.c -lm

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define ANGLE_STEP 40
#define LINES (360 * 7 / ANGLE_STEP)
int motor_lines[LINES][4];


// copy unique values in ascending order
int sort(int *output, int column)
{
    int i, j;
    int last_value = -1;
    int total = 0;


// for(i = 0; i < LINES; i++)
// {
// printf("sort %d %d\n", __LINE__, motor_lines[i][column]);
// }
    for(i = 0; i < LINES; i++)
    {
        int got_it = 0;
        int lowest = 65536;
        for(j = 0; j < LINES; j++)
        {
            int value = motor_lines[j][column];
            if(value > last_value &&
                value < lowest)
            {
                lowest = value;
                output[i] = value;
                got_it = 1;
            }
        }
        
        if(!got_it)
        {
            break;
        }
        else
        {
            last_value = output[i];
            total++;
        }
    }
    
    return total;
}



// translate hall0 & hall1 values to rows & columns
void tabulate_sensors(int hall0, int hall1)
{
    int hall0_sorted[LINES];
    int hall1_sorted[LINES];
    int hall0_total = sort(hall0_sorted, hall0);
    int hall1_total = sort(hall1_sorted, hall1);
    int i, j, k;

    printf("const uint16_t hall%d_table[] = \n{\n\t", hall0 - 1);
    for(i = 0; i < hall0_total; i++)
    {
        if((i % 8) == 0 && i != 0)
        {
            printf("\n\t");
        }
        printf("%d, ", hall0_sorted[i]);
    }
    printf("\n};\n\n");

    printf("const uint16_t hall%d_table[] = \n{\n\t", hall1 - 1);
    for(i = 0; i < hall1_total; i++)
    {
        if((i % 8) == 0 && i != 0)
        {
            printf("\n\t");
        }
        printf("%d, ", hall1_sorted[i]);
    }
    printf("\n};\n\n");

    printf("const uint8_t motor%d_table[] = \n{\n", 
        (hall0 - 1) / 2);



// rows
    for(i = 0; i < hall0_total; i++)
    {
        int want_hall0 = hall0_sorted[i];
        printf("// hall%d=%d\n\t", hall0 - 1, hall0_sorted[i]);
// columns
        for(j = 0; j < hall1_total; j++)
        {
            int want_hall1 = hall1_sorted[j];
// search for nearest sensor values
            int angle = -1;
            int nearest_distance = -1;
            for(k = 0; k < LINES; k++)
            {
                int distance = hypot(motor_lines[k][hall0] - want_hall0,
                    motor_lines[k][hall1] - want_hall1);
                if(distance < nearest_distance ||
                    nearest_distance < 0)
                {
                    angle = k * ANGLE_STEP;
                    nearest_distance = distance;
                }
            }

            if((j % 8) == 0 && j != 0)
            {
                printf("\n\t");
            }
            printf("%d, ", angle / 40);
        }
        printf("\n");
    }
    printf("\n};\n");
}

void main()
{
    char string[1024];
    int current_line = 0;
    while(!feof(stdin))
    {
        char *result = fgets(string, 1024, stdin);
        if(!result)
        {
            break;
        }
        
        if(current_line >= LINES)
        {
            printf("Too many lines\n");
        }
        else
        {
            char *ptr = strstr(string, "MOTORS:");
            if(ptr)
            {
                ptr += 8;
            }
            else
            {
// skip the line if no MOTORS: tag
                continue;
            }
            
            int angle, hall0, hall1, hall2, hall3;
            sscanf(ptr, "%d%d%d%d%d", &angle, &hall0, &hall1, &hall2, &hall3);
//            printf("%d %d %d %d %d\n", angle, hall0, hall1, hall2, hall3);
//            motor_lines[current_line][0] = angle;
            motor_lines[current_line][0] = hall0;
            motor_lines[current_line][1] = hall1;
            motor_lines[current_line][2] = hall2;
            motor_lines[current_line][3] = hall3;
            current_line++;
        }
//        printf("%s", string);
    }
    
    
    if(current_line < LINES)
    {
        printf("Expected %d lines.\n", LINES);
        return;
    }

    printf("// created by motor_table.c\n");
    tabulate_sensors(0, 1);
    tabulate_sensors(2, 3);





//     printf("const uint16_t motor_table[] = \n{\n");
//     int i;
//     for(i = 0; i < LINES; i++)
//     {
//         printf("\t%d, %d, %d, %d, \n",
//             motor_lines[i][0],
//             motor_lines[i][1],
//             motor_lines[i][2],
//             motor_lines[i][3]);
//     }
//     printf("};\n");
    
    
    
    
}




