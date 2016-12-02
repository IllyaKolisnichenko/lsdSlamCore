#include "lsdslamoutput.h"

#include <iostream>

#include "lsdslamoutputdefault.h"
#include "lsdslamoutputstorage.h"


lsdSlamOutput *lsdSlamOutput::create(lsdSlamOutput::LsdSlamOutput_ID id)
{
    lsdSlamOutput* instance;
    switch ( id )
    {
    // Make deafult
    case Default_ID:
        instance = new LsdSlamOutputDefault();
        break;

    // Make file storage
    case Storage_ID:
        instance = new lsdSlamOutputStorage();
        break;

    default:
        std::cout << "lsdSlamOutput::create()..ERROR: Wrong id.."  << std::endl;
        assert( false);
        break;
    }

    return instance;
}
