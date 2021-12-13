#include "globals.h"
#include "Request.h"

int Request::nextUid = 0;

Request::Request() {}

Request::Request(const Request& toCopy) {
    this->start = toCopy.start;
    this->end = toCopy.end;
    this->shortestDist = toCopy.shortestDist;
    this->reqTime = toCopy.reqTime;
    this->expectedOffTime = toCopy.expectedOffTime;
    this->scheduledOnTime = toCopy.scheduledOnTime;
    this->scheduledOffTime = toCopy.scheduledOffTime;
    this->status = toCopy.status;
    this->unique = toCopy.unique;
    this->allowedDelay = toCopy.allowedDelay;
    this->allowedWait = toCopy.allowedWait;
}

Request::Request(int start, int end, int reqTime) {
    this->start = start;
    this->end = end;
    this->shortestDist = -1;
    this->reqTime = reqTime;
    this->expectedOffTime = -1;
    this->scheduledOnTime = -1;
    this->scheduledOffTime = -1;
    this->status = Request::requestStatus::waiting;
    this->unique = nextUid++;
    this->allowedDelay = max_delay_sec;
    this->allowedWait = max_wait_sec;
}
