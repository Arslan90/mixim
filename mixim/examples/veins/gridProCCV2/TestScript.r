	nameFor1 <- c("# Bundle Sent", "# Bundle Received", "# Unique Bundle Received","# BundlesAtL3")
	resultFor1 <- c()
	globalMetrics <- c()
	# 1.2 : calculate the nbr of bundle sent, bundle received and bundle at L3
	for (j in nameFor1) {
	result <- data$scalars
	result <- data$scalars[ data$scalars$name == j, ]
	resultFor1 <- c(resultFor1, sum(result$value))
	cat("Total",j,":",sum(result$value),"\n")
	resultForVPA <- result[ grep("+.VPA.+", result$module),]
	cat("Total",j,"For VPA :",sum(resultForVPA$value),"\n")
	resultForVeh <- result[ grep("+.node.+", result$module),]
	cat("Total",j,"For Veh :",sum(resultForVeh$value),"\n")	
	}
	# 1.3 : calculate the delivery ratio
	DR <- 0
	if ( resultFor1[1] != 0){
		DR <- (resultFor1[3]/resultFor1[1])*100 
	}
	cat("Total Delivery Ratio (DR):",DR,"% \n")
	globalMetrics <- c(globalMetrics, DR)
	
	# 2 : calculate bundle overhead (OverAll & at VPA)
	# 2.1 : calculate OverAll Overhead
	Overhead <- 0
	if ( resultFor1[1] != 0){
		Overhead <- (resultFor1[4]/resultFor1[1])
	}
	cat("Total nbr of copy generated in the whole network (Overhead):",Overhead,"\n")
	globalMetrics <- c(globalMetrics, Overhead)
	# 2.2 : calculate Overhead For VPA
	OverheadForVPA <- 0
	if ( resultFor1[3] != 0){
		OverheadForVPA <- (resultFor1[2]/resultFor1[3])
	}
	cat("Total nbr of copy received by the VPA (Overhead For VPA):",OverheadForVPA,"\n")
	globalMetrics <- c(globalMetrics, OverheadForVPA)
