#include "IDtoObject.h"

IDtoObject::IDtoObject()
{
    ;
}

std::string IDtoObject::ConvertIDtoObject(int ID)
{
 switch(ID)
 {
     case 0:
        return "F20_20_B";
        break;

     case 1:
        return "F20_20_G";
        break;

     case 2:
        return "S40_40_B";
        break;

     case 3:
        return "S40_40_G";
        break;

     case 4:
        return "M20_100";
        break;

     case 5:
        return "M20";
        break;

     case 6:
        return "M30";
        break;

     case 7:
        return "R20";
        break;

     case 8:
        return "Bearing_Box";
        break;

     case 9:
        return "Bearing";
        break;

     case 10:
        return "Axis";
        break;

     case 11:
        return "Distance_Tube";
        break;

     case 12:
        return "Motor";
        break;

     case 13:
        return "Red_Container_Top";
        break;

     case 14:
        return "Blue_Container_Top";
        break;

     case 15:
        return "Red_Container_Front";
        break;

     case 16:
        return "Blue_Container_Front";
        break;

     case 101:
        return "F20_20_Mall";
        break;

     case 103:
        return "F40_40_Mall";
        break;

     case 105:
        return "Bolt_Mall";
        break;

     case 106:
        return "M20_Mall";
        break;

     case 107:
        return "M30_Mall";
        break;

     case 108:
        return "R20_Mall";
        break;

 }
}