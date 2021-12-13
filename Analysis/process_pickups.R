library(data.table)
library(stringr)
library(readr)
library(tidyr)

swr = function(string, nwrap=10) {
     paste(strwrap(string, width=nwrap), collapse="\n")
}
swr = Vectorize(swr)
sep = "/"
sep = "\\"
timepattern <- "[0-9]{4}_[0-9]{4}_[0-9]{4}_[0-9]{4}"
#base.path = "/ocean/projects/eng200002p/mbruchon/RidePooling/Out/"
base.path = "C:\\Code_Projects\\RidePooling\\Out\\"
files = list.files(path="C:\\Code_Projects\\RidePooling\\Out\\", pattern=timepattern, full.names=TRUE,recursive=TRUE)
timestamps <- str_extract(files, timepattern)
datestring <- max(timestamps)
#datestring <- "2021_1128_2252_4146"

filedir = paste0(base.path,datestring,sep,"Routes")
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

filedir = paste0(base.path,datestring,sep,"Pickups")
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
pickups <- unique(pickups)
setorder(pickups,time_window,time_actual,event_type,rider_id)
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

setorder(pickups,time_window,time_actual,event_type,rider_id)
pickups[,':='(occupancy = cumsum(fifelse(event_type=="on",1,-1))),by=.(car_id)]
pickups[,':='(occupancy.change = occupancy-shift(occupancy,type="lag")),by=.(car_id)]
pickups[,':='(occupancy.change.floor=pmax(occupancy.change,0,na.rm=TRUE))]
pickups[,':='(occupancy.counter=cumsum(occupancy.change.floor)),by=.(car_id)]
max.occupancy = pickups[,.(min.overfull.time = min(ifelse(occupancy>2,time_actual,Inf))),by=.(car_id)][min.overfull.time<Inf]

sharing <- pickups[,.(
     pickup.occupancy=max(ifelse(time_actual==min(time_actual),occupancy,0)),
     en.route.pickups=max(ifelse(time_actual>=min(time_actual) & time_actual <= max(time_actual),occupancy.counter,0))-
          min(ifelse(time_actual>=min(time_actual) & time_actual <= max(time_actual),occupancy.counter,0))
),by=.(rider_id,car_id)][,total.sharing:=pickup.occupancy+en.route.pickups]

rider_rollup[sharing,':='(total.sharing=total.sharing,car_id=car_id),on=.(rider_id)]
saveRDS(rider_rollup,paste0(base.path,datestring,sep,"rider_rollup.RDS"))
View(rider_rollup)

riders.private = readRDS("C:\\Users\\Matthew\\Desktop\\Ride Pooling\\rider_rollup_private.RDS")
riders.social = readRDS("C:\\Users\\Matthew\\Desktop\\Ride Pooling\\rider_rollup_social.RDS")

riders.private <- riders.private[o>=0 & d>=0]
riders.social <- riders.social[o>=0 & d>=0 & rider_id %in% riders.private$rider_id]
riders.private <- riders.private[rider_id %in% riders.social$rider_id]
riders.private <- riders.private[rider_id %in% riders.social$rider_id]

