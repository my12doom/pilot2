#include "ZLinkFile.h"
#include <modules/utils/crc16.h>

void update_crc(ZLinkFileBlock *b)
{
    b->crc = crc16(&b->size, sizeof(ZLinkFileBlock) + b->size - 4);
}

bool check_crc(const ZLinkFileBlock *b)
{
    uint16_t crc = crc16(&b->size, sizeof(ZLinkFileBlock) + b->size - 4);
    return crc == b->crc;
}
