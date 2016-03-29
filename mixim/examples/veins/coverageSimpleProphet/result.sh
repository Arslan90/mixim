#! /bin/sh
cp Coverage.r results50M/     
cp Coverage.r results100M/    
cp Coverage.r results127M/    
cp Coverage.r results150M/    
cp Coverage.r results200M/    
cp Coverage.r results250M/    
cp Coverage.r results300M/    
cp Coverage.r results50MCR/   
cp Coverage.r results100MCR/  
cp Coverage.r results127MCR/  
cp Coverage.r results150MCR/  
cp Coverage.r results200MCR/  
cp Coverage.r results250MCR/  
cp Coverage.r results300MCR/  

cd results50M/    && ~/R-2.11.1/bin/Rscript Coverage.r    
cd ../results100M/   && ~/R-2.11.1/bin/Rscript Coverage.r
cd ../results127M/   && ~/R-2.11.1/bin/Rscript Coverage.r
cd ../results150M/   && ~/R-2.11.1/bin/Rscript Coverage.r
cd ../results200M/   && ~/R-2.11.1/bin/Rscript Coverage.r
cd ../results250M/   && ~/R-2.11.1/bin/Rscript Coverage.r
cd ../results300M/   && ~/R-2.11.1/bin/Rscript Coverage.r
cd ../results50MCR/  && ~/R-2.11.1/bin/Rscript Coverage.r
cd ../results100MCR/ && ~/R-2.11.1/bin/Rscript Coverage.r
cd ../results127MCR/ && ~/R-2.11.1/bin/Rscript Coverage.r
cd ../results150MCR/ && ~/R-2.11.1/bin/Rscript Coverage.r
cd ../results200MCR/ && ~/R-2.11.1/bin/Rscript Coverage.r
cd ../results250MCR/ && ~/R-2.11.1/bin/Rscript Coverage.r 
cd ../results300MCR/ && ~/R-2.11.1/bin/Rscript Coverage.r
 