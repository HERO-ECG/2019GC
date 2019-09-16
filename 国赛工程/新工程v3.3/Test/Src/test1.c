#include <math.h>


typedef unsigned char uint8_t;
typedef unsigned int uint32_t;
typedef signed int   int32_t;
typedef signed char int8_t;
typedef signed short     int int16_t;
typedef unsigned short     int uint16_t;

typedef struct
{
    float x;
    float y;
    float z;
}Coordinate_t;

Coordinate_t AdjustTriaxialCoordinate(Coordinate_t input_coordinate, float max_component)
{
    Coordinate_t output_coordinate={0,0,0};
    Coordinate_t A0;
    float A_abs;
    float input_abs;
    int8_t sign_x=1;
    int8_t sign_y=1;
    int8_t sign_z=1;
    if((input_coordinate.x==0&&input_coordinate.y==0&&input_coordinate.z==0)||max_component<=0)
        return output_coordinate;
    if(input_coordinate.x<0)
    {
        sign_x=-1;
        input_coordinate.x=-input_coordinate.x/max_component;
    }
    else
    {
        input_coordinate.x=input_coordinate.x/max_component;
    }
    if(input_coordinate.y<0)
    {
        sign_y=-1;
        input_coordinate.y=-input_coordinate.y/max_component;
    }
    else
    {
        input_coordinate.y=input_coordinate.y/max_component;
    }
    if(input_coordinate.z<0)
    {
        sign_z=-1;
        input_coordinate.z=-input_coordinate.z/max_component;
    }
    else
    {
        input_coordinate.z=input_coordinate.z/max_component;
    }
    if(input_coordinate.y/input_coordinate.x<=1&&input_coordinate.z/input_coordinate.x<=1)
    {
        A0.x=1;
        A0.y=input_coordinate.y/input_coordinate.x;
        A0.z=input_coordinate.z/input_coordinate.x;
    }
    else if(input_coordinate.x/input_coordinate.y<=1&&input_coordinate.z/input_coordinate.y<=1)
    {
        A0.x=input_coordinate.x/input_coordinate.y;
        A0.y=1;
        A0.z=input_coordinate.z/input_coordinate.y;
    }
    else if(input_coordinate.x/input_coordinate.z<=1&&input_coordinate.y/input_coordinate.z<=1)
    {
        A0.x=input_coordinate.x/input_coordinate.z;
        A0.y=input_coordinate.y/input_coordinate.z;
        A0.z=1;
    }
    A_abs= sqrt(A0.x*A0.x+A0.y*A0.y+A0.z*A0.z);
    input_abs=sqrt(input_coordinate.x*input_coordinate.x+input_coordinate.y*input_coordinate.y+input_coordinate.z*input_coordinate.z);
    output_coordinate.x=sign_x*max_component*((input_coordinate.x*input_abs)/((input_coordinate.x+input_coordinate.y+input_coordinate.z)*A_abs));
    output_coordinate.y=sign_y*max_component*((input_coordinate.y*input_abs)/((input_coordinate.x+input_coordinate.y+input_coordinate.z)*A_abs));
    output_coordinate.z=sign_z*max_component*((input_coordinate.z*input_abs)/((input_coordinate.x+input_coordinate.y+input_coordinate.z)*A_abs));
    return output_coordinate;
}


//int main()
//{
//    Coordinate_t input_coordinate={1,10,10};
//    Coordinate_t output_coordinate=AdjustTriaxialCoordinate(input_coordinate,10);
//    printf("(%f,%f,%f),sum=%f",output_coordinate.x,output_coordinate.y,output_coordinate.z,output_coordinate.x+output_coordinate.y+output_coordinate.z);
//    return 0;

//}

