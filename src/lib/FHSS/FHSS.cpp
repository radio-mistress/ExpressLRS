#include "FHSS.h"
#include "logging.h"
#include "options.h"
#include <string.h>

#if defined(RADIO_SX127X)
#include "SX127xDriver.h"
const fhss_config_t domains[] = {
    {"AU915",  
     FREQ_HZ_TO_REG_VAL(915500000), FREQ_HZ_TO_REG_VAL(926900000), 20,
     FREQ_HZ_TO_REG_VAL(915500000), FREQ_HZ_TO_REG_VAL(926900000), 20},
    {"FCC915", 
     FREQ_HZ_TO_REG_VAL(903500000), FREQ_HZ_TO_REG_VAL(926900000), 40,
     FREQ_HZ_TO_REG_VAL(903500000), FREQ_HZ_TO_REG_VAL(926900000), 40},
    {"EU868",  
     FREQ_HZ_TO_REG_VAL(865275000), FREQ_HZ_TO_REG_VAL(869575000), 13,
     FREQ_HZ_TO_REG_VAL(865275000), FREQ_HZ_TO_REG_VAL(869575000), 13},
    {"IN866",  
     FREQ_HZ_TO_REG_VAL(865375000), FREQ_HZ_TO_REG_VAL(866950000), 4,
     FREQ_HZ_TO_REG_VAL(865375000), FREQ_HZ_TO_REG_VAL(866950000), 4},
    {"AU433",  
     FREQ_HZ_TO_REG_VAL(433420000), FREQ_HZ_TO_REG_VAL(434420000), 3,
     FREQ_HZ_TO_REG_VAL(433420000), FREQ_HZ_TO_REG_VAL(434420000), 3},
    {"EU433",  
     FREQ_HZ_TO_REG_VAL(433420000), FREQ_HZ_TO_REG_VAL(434420000), 8,
     FREQ_HZ_TO_REG_VAL(433420000), FREQ_HZ_TO_REG_VAL(434420000), 8},
    {"US433",  
     FREQ_HZ_TO_REG_VAL(433250000), FREQ_HZ_TO_REG_VAL(438000000), 8,
     FREQ_HZ_TO_REG_VAL(433250000), FREQ_HZ_TO_REG_VAL(438000000), 8},
    {"US433W", 
     FREQ_HZ_TO_REG_VAL(423500000), FREQ_HZ_TO_REG_VAL(438000000), 20,
     FREQ_HZ_TO_REG_VAL(423500000), FREQ_HZ_TO_REG_VAL(438000000), 20},
    {"433Wide_FCC915", 
     FREQ_HZ_TO_REG_VAL(433050000), FREQ_HZ_TO_REG_VAL(434790000), 20, // Radio 1: 433 Wide
     FREQ_HZ_TO_REG_VAL(902000000), FREQ_HZ_TO_REG_VAL(928000000), 40  // Radio 2: 915 US
    },
    {"144US_433Wide", 
     FREQ_HZ_TO_REG_VAL(144390000), FREQ_HZ_TO_REG_VAL(144490000), 3, // Radio 1: 144 US 
     FREQ_HZ_TO_REG_VAL(433050000), FREQ_HZ_TO_REG_VAL(434790000), 8 // Radio 2: 433 Wide
    },
    //todo  Add the ISM 2.4GHz domain for SX128X radios
   // {"ISM2G4", 
   //  FREQ_HZ_TO_REG_VAL(2400400000), FREQ_HZ_TO_REG_VAL(2479400000), 80,
   //  FREQ_HZ_TO_REG_VAL(2400400000), FREQ_HZ_TO_REG_VAL(2479400000), 80},
    // Add any additional domains as needed
};

#elif defined(RADIO_SX128X)
#include "SX1280Driver.h"

const fhss_config_t domains[] = {
    {
    #if defined(Regulatory_Domain_EU_CE_2400)
        "CE_LBT",
        FREQ_HZ_TO_REG_VAL(2400400000), // Start frequency for Radio 1
        FREQ_HZ_TO_REG_VAL(2479400000), // End frequency for Radio 1
        80, // Channel spacing for Radio 1
        FREQ_HZ_TO_REG_VAL(2400400000), // Start frequency for Radio 2 
        FREQ_HZ_TO_REG_VAL(2479400000), // End frequency for Radio 2 
        80  // Channel spacing for Radio 2 
    #elif defined(Regulatory_Domain_ISM_2400)
        "ISM2G4",
        FREQ_HZ_TO_REG_VAL(2400400000), // Start frequency for Radio 1
        FREQ_HZ_TO_REG_VAL(2479400000), // End frequency for Radio 1
        80, // Channel spacing for Radio 1
        FREQ_HZ_TO_REG_VAL(2400400000), // Start frequency for Radio 2 
        FREQ_HZ_TO_REG_VAL(2479400000), // End frequency for Radio 2 
        80  // Channel spacing for Radio 2 
    #endif
    }
};
#endif


// Our table of FHSS frequencies. Define a regulatory domain to select the correct set for your location and radio
const fhss_config_t *FHSSconfig;

// Actual sequence of hops as indexes into the frequency list
uint8_t FHSSsequence[256];
// Which entry in the sequence we currently are on
volatile uint8_t FHSSptr;
// Channel for sync packets and initial connection establishment
uint_fast8_t sync_channel;
// Offset from the predefined frequency determined by AFC on Team900 (register units)
int32_t FreqCorrection;
int32_t FreqCorrection_2;

uint32_t freq_spread;

void FHSSrandomiseFHSSsequence(const uint32_t seed)
{
    FHSSconfig = &domains[firmwareOptions.domain];
    DBGLN("Setting %s Mode", FHSSconfig->domain);
    DBGLN("Number of FHSS frequencies for Radio 1 = %u", FHSSconfig->freq_count_1);
    DBGLN("Number of FHSS frequencies for Radio 2 = %u", FHSSconfig->freq_count_2);

    sync_channel = (FHSSconfig->freq_count_1 / 2) + 1; // Assuming sync channel is based on Radio 1
    DBGLN("Sync channel = %u", sync_channel);

    freq_spread = (FHSSconfig->freq_stop_1 - FHSSconfig->freq_start_1) * FREQ_SPREAD_SCALE / (FHSSconfig->freq_count_1 - 1);

    // reset the pointer (otherwise the tests fail)
    FHSSptr = 0;
    rngSeed(seed);

    // initialize the sequence array
    for (uint16_t i = 0; i < FHSSgetSequenceCount_1(); i++)
    {
        if (i % FHSSconfig->freq_count_1 == 0) {
            FHSSsequence[i] = sync_channel;
        } else if (i % FHSSconfig->freq_count_1 == sync_channel) {
            FHSSsequence[i] = 0;
        } else {
            FHSSsequence[i] = i % FHSSconfig->freq_count_1;
        }
    }

    for (uint16_t i=0; i < FHSSgetSequenceCount_1(); i++)
    {
        // if it's not the sync channel
        if (i % FHSSconfig->freq_count_1 != 0)
        {
            uint8_t offset = (i / FHSSconfig->freq_count_1) * FHSSconfig->freq_count_1; // offset to start of current block
            uint8_t rand = rngN(FHSSconfig->freq_count_1-1)+1; // random number between 1 and FHSS_FREQ_CNT

            // switch this entry and another random entry in the same block
            uint8_t temp = FHSSsequence[i];
            FHSSsequence[i] = FHSSsequence[offset+rand];
            FHSSsequence[offset+rand] = temp;
        }
    }

    // output FHSS sequence
    for (uint16_t i=0; i < FHSSgetSequenceCount_1(); i++)
    {
        DBG("%u ",FHSSsequence[i]);
        if (i % 10 == 9)
            DBGCR;
    }
    DBGCR;
}

bool isDomain868()
{
    return strcmp(FHSSconfig->domain, "EU868") == 0;
}