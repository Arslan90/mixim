# loading omnetpp library
library(omnetpp)
# set the output file
sink("results.txt")
# loading dataset in data var
data <- loadDataset("General-*.sca")

# 0 : calculate L3 msgs sent/received
# 0.1 : calculate the nbr of L3 msgs sent
nbrL3Sent <- data$scalars[ data$scalars$name == "# L3Sent", ]
cat("\n total des msgs L3 envoyes :",sum(nbrL3Sent$value))
# 0.2 : calculate the nbr of L3 msgs received
nbrL3Received <- data$scalars[ data$scalars$name == "# L3Received", ]
cat("\n total des msgs L3 recus :",sum(nbrL3Received$value))

# 1 : calculate bundle reception in %
# 1.1 : calculate the nbr of simulation 
nbrSim <- nlevels(data$fileruns$runid)
cat("\n nombre de simulation :", nbrSim)
# 1.2 : calculate the nbr of bundle sent by vehs
nbrSent <- data$scalars
nbrSent <- data$scalars[ grep("scenarioOpp.VPA.+", data$scalars$module),]
nbrSent <- nbrSent[ nbrSent$name == "# Bundle Sent", ]
cat("\n nombre de simulation pris en compte pour les bundles envoyes par vehs :",nlevels(nbrSent$runid))
cat("\n total des bundles envoyes par vehs :",sum(nbrSent$value))
# 1.3 : calculate the nbr of unique bundle received by vpa
nbrReceived <- data$scalars
nbrReceived <- data$scalars[ grep("scenarioOpp.VPA.+", data$scalars$module),]
nbrReceived <- nbrReceived[ nbrReceived$name == "# Unique Bundle Received", ]
cat("\n nombre de simulation pris en compte pour les bundles recues par VPA :",nlevels(nbrReceived$runid))
cat("\n total des bundles uniques recues par VPA :",sum(nbrReceived$value))
# 1.4 : calculte the delivery in pourcentage
totalSent <- sum(nbrSent$value)
totalReceived <- sum(nbrReceived$value)
if (nlevels(nbrSent$runid) == nlevels(nbrReceived$runid)){
delivery <- (totalReceived/totalSent)*100
cat("\n le ratio de livraison des bundles :", delivery) 
} else {
cat("\n le ratio de livraison doit se calculer manuellement")
}
# 1.5 : calculate the total number of bundle received at L3 layer by all nodes

nbrTotalReceivedAtL3 <- data$scalars
#nbrTotalReceivedAtL3 <- data$scalars[ grep("scenarioOpp.node.+", data$scalars$module),]
nbrTotalReceivedAtL3 <- nbrTotalReceivedAtL3[ nbrTotalReceivedAtL3$name == "# Bundles at L3", ]
cat("\n nombre de simulation pris en compte pour les bundles envoyes par vehs :",nlevels(nbrTotalReceivedAtL3$runid))
cat("\n total des bundles recus par la couche reseau pour tous les noeuds :",sum(nbrTotalReceivedAtL3$value))

# 2 : calculate the nbr of copy for each bundle
# 2.1 : calculate the nbr of bundle received by vpa
nbrMultiReceived <- data$scalars
nbrMultiReceived <- data$scalars[ grep("scenarioOpp.VPA.+", data$scalars$module),]
nbrMultiReceived <- nbrMultiReceived[ nbrMultiReceived$name == "# Bundle Received", ]
cat("\n nombre de simulation pris en compte pour les bundles recues par VPA :",nlevels(nbrMultiReceived$runid))
cat("\n total des bundles recues par VPA :",sum(nbrMultiReceived$value))
# 2.2 : calculte the nbr of copy for each simulation
totalMultiReceived <- sum(nbrMultiReceived$value)
totalReceived <- sum(nbrReceived$value)
if (nlevels(nbrMultiReceived$runid) == nlevels(nbrReceived$runid)){
nbrCopy <- (totalMultiReceived/totalReceived)
cat("\n le nombre de copies recus par VPA :", nbrCopy) 
} else {
cat("\n le nombre de copies recus par VPA doit se calculer manuellement")
}
# 2.3 : calculte the nbr of copy for each simulation
totalMultiReceived <- sum(nbrTotalReceivedAtL3$value)
totalReceived <- sum(nbrReceived$value)
if (nlevels(nbrMultiReceived$runid) == nlevels(nbrReceived$runid)){
nbrCopy <- (totalMultiReceived/totalReceived)
cat("\n le nombre de copies generes :", nbrCopy) 
} else {
cat("\n le nombre de copies generes doit se calculer manuellement")
}

# post process : making histogram data & filtring histograms with no entries added
histogram <- merge(data$statistics, data$fields)
delayHistogramID <- histogram[ histogram$name == "Delays for 1st Copy" & histogram$fieldname =="count" & histogram$fieldvalue != 0 , ]
delayHistogram <- histogram[ histogram$name == "Delays for 1st Copy" & histogram$resultkey %in% delayHistogramID$resultkey, ]

# 3 : calculating bundle delay reception 
# 3.1 : calculating mean bundle delay reception 
bndlDelay_mean <- delayHistogram[ delayHistogram$name == "Delays for 1st Copy" & delayHistogram$fieldname =="mean", ]
cat("\n delai moyen de reception des bundles pour la premiere copie:", mean(bndlDelay_mean$fieldvalue))
cat("\n nombre de simulation pris en compte pour les delais de reception :",nlevels(bndlDelay_mean$runid))
# 3.2 : calculating max bundle delay reception 
bndlDelay_max <- delayHistogram[ delayHistogram$name == "Delays for 1st Copy" & delayHistogram$fieldname =="max", ]
cat("\n delai max de reception des bundles pour la premiere copie:", max(bndlDelay_max$fieldvalue))
# 3.3 : calculating mean of all stddev bundle delay reception 
bndlDelay_stddev <- delayHistogram[ delayHistogram$name == "Delays for 1st Copy" & delayHistogram$fieldname =="stddev", ]
cat("\n moyenne des stddev des delais de reception des bundles  :", mean(bndlDelay_stddev$fieldvalue))
# 3.4 : calculating the new stddev bundle delay reception 
Mean <- mean(bndlDelay_mean$fieldvalue)
List <- bndlDelay_mean$fieldvalue
Sum <- 0
N <- nrow(bndlDelay_mean)
for (i in List){
Sum <- Sum + (i-Mean)^2
}
Variance <- Sum / N
Stddev <- sqrt(Variance)
cat("\n stddev des delais de reception des bundles pour la premiere copie:", Stddev)

# post process : making histogram data & filtring histograms with no entries added
histogram <- merge(data$statistics, data$fields)
hopHistogramID <- histogram[ histogram$name == "HopCount for 1st Copy" & histogram$fieldname =="count" & histogram$fieldvalue != 0 , ]
hopHistogram <- histogram[ histogram$name == "HopCount for 1st Copy" & histogram$resultkey %in% hopHistogramID$resultkey, ]

# 4 : calculating hop count 
# 4.1 : calculating mean hop count
hopCount_mean <- hopHistogram[ hopHistogram$name == "HopCount for 1st Copy" & hopHistogram$fieldname =="mean", ]
cat("\n nombre de saut moyen pour la premiere copie:", mean(hopCount_mean$fieldvalue))
cat("\n nombre de simulation pris en compte pour les delais de reception :",nlevels(hopCount_mean$runid))
# 4.2 : calculating mean hop count 
hopCount_max <- hopHistogram[ hopHistogram$name == "HopCount for 1st Copy" & hopHistogram$fieldname =="max", ]
cat("\n nombre de saut max pour la premiere copie:", max(hopCount_max$fieldvalue))
# 4.3 : calculating stddev hop count 
hopCount_stddev <- hopHistogram[ hopHistogram$name == "HopCount for 1st Copy" & hopHistogram$fieldname =="stddev", ]
cat("\n moyenne des stddev du nombre de saut :", mean(hopCount_stddev$fieldvalue))
# 4.4 : calculating the new stddev bundle delay reception 
Mean <- mean(hopCount_mean$fieldvalue)
List <- hopCount_mean$fieldvalue
Sum <- 0
N <- nrow(hopCount_mean)
for (i in List){
Sum <- Sum + (i-Mean)^2
}
Variance <- Sum / N
Stddev <- sqrt(Variance)
cat("\n stddev des nombres de saut pour la premiere copie:", Stddev)


barsVector <- c(0,0,0,0,0)
for (i in 1:5){
scalarName1 <- paste("#repeated",i)
scalarName <- paste("Bars for",scalarName1,sep=" ")
nbrRContacts <- data$scalars[ data$scalars$name == scalarName, ]
cat("\ncurrent scalar name:",scalarName)
cat(" - total values for it:",sum(nbrRContacts$value))
barsVector[i] <- sum(nbrRContacts$value)
}
# Bars Chart for nbr recontact
barplot(barsVector)
totalBarsVector <- sum(barsVector)
densVector <- c(0,0,0,0,0)
for (i in 1:5){
densVector[i] <- barsVector[i] / totalBarsVector
}
# PDF Chart for nbr recontact
plot(densVector, ylim=c(0,1))
lines(seq(1:5),densVector, col="red")
# CDF Chart for nbr recontact
sum <- 0
cumVector <- c(0,0,0,0,0)
for (i in 1:5) { 
sum <- sum + densVector[i]; 
cumVector[i] <- sum 
}
lines(seq(1:5),cumVector, col="blue")

lineNames <- c()
colors <- rainbow(10)
ploted <- FALSE
for (i in 1:5){
scalarName1 <- paste("#repeated",i)
scalarName <- paste("Bars for",scalarName1,sep=" ")
#nbrRContacts <- data$scalars[ data$scalars$name == scalarName, ]
#cat("\ncurrent scalar name:",scalarName)
#cat(" - total values for it:",sum(nbrRContacts$value))
#barsVector[i] <- sum(nbrRContacts$value)
scalarNamePart2 <- c("LQ1","G1LQ5","G5LQ20","G20LQ50","G50LQ100","G100LQ500","G500")
	contactVector<- c(0,0,0,0,0,0,0)
	intercontactVector<- c(0,0,0,0,0,0,0)

	for (j in scalarNamePart2) {
		index <- match(j,scalarNamePart2)
		scalarName <- paste("Contact CDF",j,sep=" ")
		scalarName <- paste(scalarName,scalarName1,sep=" ")
		contactDurationForCDF <- data$scalars[ data$scalars$name == scalarName, ]
		cat("\nTotal Value for scalar name:",scalarName, "is = ",sum(contactDurationForCDF$value))
		contactVector[index] <- sum(contactDurationForCDF$value)
		scalarName <- paste("InterContact CDF",j,sep=" ")
		scalarName <- paste(scalarName,scalarName1,sep=" ")
		intercontactDurationForCDF <- data$scalars[ data$scalars$name == scalarName, ]
		cat("\nTotal Value for scalar name:",scalarName, "is = ",sum(intercontactDurationForCDF$value))
		intercontactVector[index] <- sum(intercontactDurationForCDF$value)
	}
	
	total <- sum(contactVector)
	sumContact <- 0
	string <- paste("Contact CDF for #repeated",i)
	lineNames <- c(lineNames, string)
	cat("\n",string)
	for (j in scalarNamePart2) {
		index <- match(j,scalarNamePart2)
		sumContact <- sumContact + contactVector[index]
		contactVector[index] <- sumContact / total
		cat(": ",contactVector[index])
	}

	if (!ploted) {
	plot(contactVector,ylim=c(0,1),type = "n")
	lines(seq(1:7),contactVector,col=colors[(i-1)*2])
	ploted <- TRUE				
	} else {
	lines(seq(1:7),contactVector,col=colors[(i-1)*2])
	}

	total <- sum(intercontactVector)
	sumInterContact <- 0
	string <- paste("InterContact CDF for #repeated",i)
	lineNames <- c(lineNames, string)
	cat("\n",string)
	for (j in scalarNamePart2) {
		index <- match(j,scalarNamePart2)
		sumInterContact <- sumInterContact + intercontactVector[index]
		intercontactVector[index] <- sumInterContact / total
		cat(": ",intercontactVector[index])
	}

	if (!ploted) {
	plot(intercontactVector,ylim=c(0,1),type = "n")
	lines(seq(1:7),intercontactVector,col=colors[(i-1)*2+1], type = 'b')
	ploted <- TRUE				
	} else {
	lines(seq(1:7),intercontactVector,col=colors[(i-1)*2+1], type = 'b')
	}
}
par(xpd=TRUE)
legend("right",title="CDF Contact & InterContact",legend=lineNames,fill=rainbow(10))

sink()
browser()
