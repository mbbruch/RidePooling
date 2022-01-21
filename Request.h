#pragma once

class Request {
private:

public:
    enum class requestStatus { waiting=3, onBoard=4, droppedOff=5 };
    requestStatus getStatus() const { return status; };
    void setStatus(requestStatus newStatus);
    void fixStatus(int nowTime);
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
