#pragma once

class Request {
public:
    enum requestStatus { waiting, onBoard, droppedOff };
    int start, end;
    int shortestDist;
    int reqTime, expectedOffTime;
    int scheduledOnTime, scheduledOffTime;
    requestStatus status;
    int unique;

    static int nextUid;

    Request();
    Request(const Request& toCopy);

    Request(int start, int end, int reqTime);
};
