
#include "sacp_frame.h"

#include <iostream>


sacpAttribute_t attr[5] = { 0 };




int main()
{
    sacpAttributeSetFloat(&attr[0], 100, 0.123);
    sacpAttributeSetUint32(&attr[1], 101, 123456);
    sacpAttributeSetInt16(&attr[2], 102, -1000);
    sacpAttributeSetUint8(&attr[3], 103, 255);
    sacpAttributeSetStatus(&attr[4], 104, 25);

    uint8_t frame[CONFIG_SACP_MAX_FRAME_SIZE];
    int seq = 23;
    int pri = 1;
    int op = SACP_OP_WRITE;

    // 打包成一个帧
    int frameSize = sacpMakeFrame(seq, SACP_FRAME_CONTROL(pri, op), attr, 5, frame, sizeof(frame));
    std::cout << "call sacpMakeFrame return " << frameSize << std::endl;

    {
        char buffer[512];
        std::cout << "MAKE FRAME:" << hexStringEncode(buffer, sizeof(buffer), frame, frameSize) << std::endl;
    }

    // 将帧解析出来 
    sacpFrameInfo_t *frameInfo = sacpFrameParse(frame, frameSize);

    if (frameInfo)
    {
        sacpFrameInfoDump("PARSE FRAME:", frameInfo);

        sacpFrameInfoFree(frameInfo);
    }

    return 0;
}

