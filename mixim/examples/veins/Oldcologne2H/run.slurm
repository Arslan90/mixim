#!/bin/bash
#SBATCH --job-name=cologne2H
#SBATCH --output=slurm.out
#SBATCH --error=slurm.err
#SBATCH --nodes=1
#SBATCH --mail-type=ALL
#SBATCH --mail-user=hc.arslan90@gmail.com
  
srun -l vanet_sim.sh
