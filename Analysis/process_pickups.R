library(data.table)
library(stringr)
library(readr)
library(tidyr)

swr = function(string, nwrap=10) {
     paste(strwrap(string, width=nwrap), collapse="\n")
}
swr = Vectorize(swr)
timepattern <- "[0-9]{4}_[0-9]{4}_[0-9]{4}_[0-9]{4}"
files = list.files(path="C:\\Code_Projects\\RidePooling\\Out\\", pattern=timepattern, full.names=TRUE,recursive=TRUE)
timestamps <- str_extract(files, timepattern)
datestring <- max(timestamps)
#datestring <- "2021_1017_1653_5081"

filedir = paste0("C:\\Code_Projects\\RidePooling\\Out\\",datestring,"\\Routes")
pattern = "^[0-9]{1,4}.csv"
files = list.files(path=filedir, pattern=pattern, full.names=TRUE,recursive=TRUE)
ids <- str_extract(files, "[0-9]{1,4}.csv")
ids <- as.numeric(substr(ids,1,str_length(ids)-4))
routes=lapply(files, fread, header=FALSE)
for (i in 1:length(routes)){
     routes[[i]]$car_id <- ids[i]
}
routes <- rbindlist(routes)
colnames(routes) <- c("time_window","time","node","car_id")
setorder(routes,car_id,time)
#TODO merge distances on from edge list
#routes[,cumulative_distance:=cumsum(distance),by=car_id]

filedir = paste0("C:\\Code_Projects\\RidePooling\\Out\\",datestring,"\\Pickups")
pattern = "*.csv"
files = list.files(path=filedir, pattern=pattern, full.names=TRUE,recursive=TRUE)
ids <- str_extract(files, "[0-9]{1,4}.csv")
ids <- as.numeric(substr(ids,1,str_length(ids)-4))
pickups=lapply(files, fread, header=FALSE)
for (i in 1:length(pickups)){
     pickups[[i]]$car_id <- ids[i]
}
pickups <- rbindlist(pickups)
colnames(pickups) <- c("time_window","rider_id","event_type","node",
           "time_actual","time_ideal","car_id")

rider_rollup <- pickups[,.(o=max(fifelse(event_type=="on",node,as.numeric(NA)),na.rm=TRUE),
                           d=max(fifelse(event_type=="off",node,as.numeric(NA)),na.rm=TRUE),
                           tRequest=min(fifelse(event_type=="on",time_ideal,as.numeric(NA)),na.rm=TRUE),
                           tPickup=max(fifelse(event_type=="on",time_actual,as.numeric(NA)),na.rm=TRUE),
                           tDropoffIdeal=min(fifelse(event_type=="off",time_ideal,as.numeric(NA)),na.rm=TRUE),
                           tDropoff=max(fifelse(event_type=="off",time_actual,as.numeric(NA)),na.rm=TRUE)),
                           by=rider_id]
rider_rollup[,':='(ideal_travel_time=tDropoffIdeal-tRequest,travel_time=tDropoff-tPickup)]
rider_rollup[,':='(wait=tPickup-tRequest,delay=tDropoff-tDropoffIdeal)][,total_time_lost:=wait+delay]
View(rider_rollup[o==-Inf | d==-Inf][order(rider_id)])