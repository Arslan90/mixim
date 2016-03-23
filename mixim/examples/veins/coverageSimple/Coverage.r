# loading omnetpp library
library(omnetpp)
#library(plyr)
globalFileName <- "testScript.csv"
#sink(file = globalFileName, append = FALSE)
#sink(type = "message")
cat("ConfigName; Parameters; DeliveryRatio-mean; DeliveryRatio-sd; L3DR-mean; L3DR-sd; L2DR-mean; L2DR-sd; L2LostPackets; L2DroppedPackets; ",file = globalFileName, append = FALSE)

cat("\n",file = globalFileName, append = TRUE)
nbrConfig <- 1
nbrRepeatPerConfig <- 20
globalResults <- array(0, dim=c(nbrConfig,10))
#par(mfrow=c(3,2))
for (i in 1:nbrConfig) {
	configResults <- array(0, dim=c(1,24))
	#sink(file = globalFileName, append = TRUE)
	#sink(type = "message")
	# 0 : Preparing data handling and saving parameters
	fileName <- paste("configuration",i,sep="")
	fileName <- paste(fileName,".txt",sep="")
	#cat(fileName,";",file = globalFileName, append = TRUE)
	globalResults[i,1] <- fileName
	#sink(file = fileName, append = FALSE)
	#sink(type = "message")
	first_index <- (i-1)*nbrRepeatPerConfig
	last_index <- (i-1)*nbrRepeatPerConfig+(nbrRepeatPerConfig-1)
	scalarsNames <- c()
	for (j in first_index:last_index){
		name <- paste("General-",j,sep="")
		name <- paste(name,".sca",sep="")
		#cat(name,"\n")
		scalarsNames <- c(scalarsNames, name)
	}
	cat("Loaded Scalars:",scalarsNames,"\n",file = fileName, append = FALSE)
	data <- loadDataset(scalarsNames)
	measurement <- data$runattrs[ data$runattrs$attrname=="iterationvars", ]
	#cat(write.table(measurement),"\n",file = fileName, append = TRUE)
	cat("Parameters:",as.character(measurement[1,3]),"\n",file = fileName, append = TRUE)
	
	# 1 : calculate bundle reception in %
	# 1.1 : calculate the nbr of simulation 
	nbrSim <- nlevels(data$fileruns$runid)
	cat("Nombre de simulation:", nbrSim,"\n",file = fileName, append = TRUE)
	nameFor1 <- c("# Bundle Sent", "# Bundle Received", "# Unique Bundle Received","# Bundles at L3")
	resultFor1 <- c()
	globalMetrics <- c()
	repeatResults <- array(0, dim=c(nbrRepeatPerConfig,4))
	# 1.2 : calculate the nbr of bundle sent, bundle received and bundle at L3
	index <- 0
	for (j in nameFor1) {
	index <- index+1
	result <- data$scalars
	result <- data$scalars[ data$scalars$name == j, ]
	resultFor1 <- c(resultFor1, sum(result$value))
	cat("Total",j,":",sum(result$value),"\n",file = fileName, append = TRUE)
	resultForVPA <- result[ grep("+.VPA.+", result$module),]
	cat("Total",j,"For VPA :",sum(resultForVPA$value),"\n",file = fileName, append = TRUE)
	resultForVeh <- result[ grep("+.node.+", result$module),]
	cat("Total",j,"For Veh :",sum(resultForVeh$value),"\n",file = fileName, append = TRUE)
		for (k in 0:(nbrRepeatPerConfig-1)) {
		fileForRepeat <- data$runattrs [ data$runattrs$attrname =="repetition" & data$runattrs$attrvalue == k,]
		dataPerRepeat <- data$scalars[ data$scalars$name == j & data$scalars$runid ==fileForRepeat$runid,  ]
		repeatResults[k+1,index] <- sum(dataPerRepeat$value)
		}	
	}
	# 1.3 : calculate the delivery ratio
	cat("\n##### Delivery Ratio #####\n",file = fileName, append = TRUE)
	metric <- 1
	DR <- 0
	if ( resultFor1[1] != 0){
		DR <- (resultFor1[3]/resultFor1[1])*100
		DRresults <- c()
		for (k in 1:nbrRepeatPerConfig) {
			DRresults <- c(DRresults, repeatResults[k,3]/repeatResults[k,1]*100)
		}
		configResults[1,(metric-1)*4+1] <- min(DRresults)
		configResults[1,(metric-1)*4+2] <- max(DRresults)
		configResults[1,(metric-1)*4+3] <- mean(DRresults)
		configResults[1,(metric-1)*4+4] <- sd(DRresults)
		
		cat("Delivery Ratio-min",min(DRresults),"% \n",file = fileName, append = TRUE)
		cat("Delivery Ratio-max",max(DRresults),"% \n",file = fileName, append = TRUE)
		cat("Delivery Ratio-mean",mean(DRresults),"% \n",file = fileName, append = TRUE)
		cat("Delivery Ratio-sd",sd(DRresults),"% \n",file = fileName, append = TRUE)
	}
	
	cat("Total Delivery Ratio (DR):",DR,"% \n",file = fileName, append = TRUE)
	globalMetrics <- c(globalMetrics, DR)
	for (k in 1:2){
		globalResults[i,(metric)*2+k] <- configResults[1,(metric-1)*4+(k+2)]
	}
	
	# 7 : calculate L3 msg reception in %
	# 7.1 : calculate the nbr of simulation 
	nbrSim <- nlevels(data$fileruns$runid)
	cat("\nNombre de simulation:", nbrSim,"\n",file = fileName, append = TRUE)
	nameFor1 <- c("# L3Sent", "# L3Received")
	resultFor1 <- c()
	repeatResults <- array(0, dim=c(nbrRepeatPerConfig,2))
	# 7.2 : calculate the nbr of L3 msg sent/received
	index <- 0
	for (j in nameFor1) {
	index <- index+1
	result <- data$scalars
	result <- data$scalars[ data$scalars$name == j, ]
	resultFor1 <- c(resultFor1, sum(result$value))
	cat("Total",j,":",sum(result$value),"\n",file = fileName, append = TRUE)
	resultForVPA <- result[ grep("+.VPA.+", result$module),]
	cat("Total",j,"For VPA :",sum(resultForVPA$value),"\n",file = fileName, append = TRUE)
	resultForVeh <- result[ grep("+.node.+", result$module),]
	cat("Total",j,"For Veh :",sum(resultForVeh$value),"\n",file = fileName, append = TRUE)
		for (k in 0:(nbrRepeatPerConfig-1)) {
		fileForRepeat <- data$runattrs [ data$runattrs$attrname =="repetition" & data$runattrs$attrvalue == k,]
		dataPerRepeat <- data$scalars[ data$scalars$name == j & data$scalars$runid ==fileForRepeat$runid,  ]
		repeatResults[k+1,index] <- sum(dataPerRepeat$value)
		}	
	}
	# 7.3 : calculate the delivery ratio for L3 msg
	cat("\n##### L3 Delivery Ratio #####\n",file = fileName, append = TRUE)
	metric <- 2
	DR <- 0
	if ( resultFor1[1] != 0){
		DR <- (resultFor1[2]/resultFor1[1])*100
		DRresults <- c()
		for (k in 1:nbrRepeatPerConfig) {
			DRresults <- c(DRresults, repeatResults[k,2]/repeatResults[k,1]*100)
		}
		configResults[1,(metric-1)*4+1] <- min(DRresults)
		configResults[1,(metric-1)*4+2] <- max(DRresults)
		configResults[1,(metric-1)*4+3] <- mean(DRresults)
		configResults[1,(metric-1)*4+4] <- sd(DRresults)
		
		cat("L3 Delivery Ratio-min",min(DRresults),"% \n",file = fileName, append = TRUE)
		cat("L3 Delivery Ratio-max",max(DRresults),"% \n",file = fileName, append = TRUE)
		cat("L3 Delivery Ratio-mean",mean(DRresults),"% \n",file = fileName, append = TRUE)
		cat("L3 Delivery Ratio-sd",sd(DRresults),"% \n",file = fileName, append = TRUE)
	}
	cat("Total L3 Delivery Ratio (DR):",DR,"% \n",file = fileName, append = TRUE)
	globalMetrics <- c(globalMetrics, DR)
	for (k in 1:2){
		globalResults[i,(metric)*2+k] <- configResults[1,(metric-1)*4+(k+2)]
	}
	
	# 8 : calculate L3 msg reception in %
	# 8.1 : calculate the nbr of simulation 
	nbrSim <- nlevels(data$fileruns$runid)
	cat("\nNombre de simulation:", nbrSim,"\n",file = fileName, append = TRUE)
	nameFor1 <- c("statsSentPackets", "statsReceivedBroadcasts", "statsLostPackets", "statsDroppedPackets")
	resultFor1 <- c()
	repeatResults <- array(0, dim=c(nbrRepeatPerConfig,4))
	# 8.2 : calculate the nbr of L3 msg sent/received
	index <- 0
	for (j in nameFor1) {
	index <- index+1
	result <- data$scalars
	result <- data$scalars[ data$scalars$name == j, ]
	resultFor1 <- c(resultFor1, sum(result$value))
	cat("Total",j,":",sum(result$value),"\n",file = fileName, append = TRUE)
	resultForVPA <- result[ grep("+.VPA.+", result$module),]
	cat("Total",j,"For VPA :",sum(resultForVPA$value),"\n",file = fileName, append = TRUE)
	resultForVeh <- result[ grep("+.node.+", result$module),]
	cat("Total",j,"For Veh :",sum(resultForVeh$value),"\n",file = fileName, append = TRUE)
		for (k in 0:(nbrRepeatPerConfig-1)) {
		fileForRepeat <- data$runattrs [ data$runattrs$attrname =="repetition" & data$runattrs$attrvalue == k,]
		dataPerRepeat <- data$scalars[ data$scalars$name == j & data$scalars$runid ==fileForRepeat$runid,  ]
		repeatResults[k+1,index] <- sum(dataPerRepeat$value)
		}	
	}
	# 8.3 : calculate the delivery ratio for L3 msg
	cat("\n##### L2 Delivery Ratio #####\n",file = fileName, append = TRUE)
	metric <- 3
	DR <- 0
	if ( resultFor1[1] != 0){
		DR <- (resultFor1[2]/resultFor1[1])*100
		DRresults <- c()
		for (k in 1:nbrRepeatPerConfig) {
			DRresults <- c(DRresults, repeatResults[k,2]/repeatResults[k,1]*100)
		}
		configResults[1,(metric-1)*4+1] <- min(DRresults)
		configResults[1,(metric-1)*4+2] <- max(DRresults)
		configResults[1,(metric-1)*4+3] <- mean(DRresults)
		configResults[1,(metric-1)*4+4] <- sd(DRresults)
		
		cat("L3 Delivery Ratio-min",min(DRresults),"% \n",file = fileName, append = TRUE)
		cat("L3 Delivery Ratio-max",max(DRresults),"% \n",file = fileName, append = TRUE)
		cat("L3 Delivery Ratio-mean",mean(DRresults),"% \n",file = fileName, append = TRUE)
		cat("L3 Delivery Ratio-sd",sd(DRresults),"% \n",file = fileName, append = TRUE)
	}
	
	cat("Total L2 Delivery Ratio (DR):",DR,"% \n",file = fileName, append = TRUE)
	globalMetrics <- c(globalMetrics, DR)
	globalMetrics <- c(globalMetrics, resultFor1[3])
	globalMetrics <- c(globalMetrics, resultFor1[4])
	
	for (k in 1:4){
		globalResults[i,(metric)*2+k] <- configResults[1,(metric-1)*4+(k+2)]
	}


	globalResults[i,2] <- basename(getwd())
	for (k in 1:10){
		cat(globalResults[i,k],";",file = globalFileName, append = TRUE)
	}
	cat("\n",file = globalFileName, append = TRUE)
}
