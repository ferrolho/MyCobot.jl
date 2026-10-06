#!/bin/bash
# Run Julia code from stdin on the Pi with the plush helpers loaded.
ssh raspberrypi5 'cat > ~/scratch/plush/cmd.jl && cd ~/myCobot/mycobot-280-lab && ~/.juliaup/bin/julia --project=. -e "include(\"/home/henrique/scratch/plush/plush.jl\"); include(\"/home/henrique/scratch/plush/cmd.jl\")"'
