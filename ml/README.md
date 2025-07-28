# ML Training Readme

Reinforcement learning is done in Unity 2022.3.16f1 with ML-Agents.

## Arena and units
- The arena is a simple quad with size 1.5 m x 1 m.
- Scale: 1 Unity unit represents 10 cm.

## Agents and dynamics
- Each rover is represented by a cylinder that is 10 cm wide.
- Movement is controlled by code using standard differential-drive equations.
- Horizontal slip is restricted in the dynamics.

## Training setup
- ML-Agents installed in a conda environment.
- Python 3.10 was used for training.

## Experiments tried

1. **Simple potential-based rewards**
   Distance difference was used for the main reward. A small "being alive" reward was given to Jerry, and a small "Jerry alive" penalty was given to Tom. Both rovers had trouble finding each other, and Jerry often ran outside the area and terminated.

2. **Potential + distance reward**
   In addition to distance difference, a distance reward was added. This improved learning speed, but it did not stop Jerry from leaving the area.

3. **Two-stage training, potential + distance -> survival only**
   Same as above at first, then after a set time all distance and potential rewards were removed for Jerry to make it focus on survival. This did not increase Jerry's survival time. Jerry still tended to exit the area instead of surviving until being caught.

4. **Two-stage training with noise injection**
   Real-world tests showed enough noise to break the learned policies. Noise was injected into the model inputs to increase robustness. This is under active exploration.

## Notes and status

**Warning:** This part is highly experimental. Training models that transfer to real-life is difficult, and this project has not yet bridged the gap. Any contributions or ideas are welcome.
