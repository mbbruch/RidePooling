#pragma once

class Request {
public:
    enum class requestStatus { waiting, onBoard, droppedOff };
    int start, end;
    int shortestDist;
    int reqTime, expectedOffTime;
    int scheduledOnTime, scheduledOffTime;
    int allowedDelay;
    int allowedWait;
    requestStatus status;
    int unique;

    static int nextUid;

    Request();
    Request(const Request& toCopy);

    Request(int start, int end, int reqTime);
};
