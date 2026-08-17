# Smart Car Tracking System (Self-Balancing Unicycle)
> Embedded vision-based autonomous vehicle — Anhui Provincial First Prize
## Tech Stack
- **MCU**: Infineon TC377 | **IDE**: AURIX™ Development Studio | **Language**: C
- **Algorithms**: Fixed-Threshold Binarization, PID Control
- **Base Library**: Seekfree_TC377_Opensource_Library
## My Work
- Implemented **fixed-threshold binarization** for track extraction
- Developed **white-point tracking** (connected-component based) to locate the nearest track region and calculate deviation
- Evaluated Otsu's Method but chose fixed-threshold for better real-time stability
- Implemented **PID control** for steering and self-balancing (Pitch/Roll angle control)
## Award
- Smart Car Competition **Anhui Provincial First Prize (Self-Balancing Unicycle Category)**
## Acknowledgements
Developed based on [Seekfree_TC377_Opensource_Library](https://github.com/seekfree/Seekfree_TC377_Opensource_Library).
**Note:** Some auxiliary code was generated with AI assistance. Hardware and other major components were completed by teammates.


