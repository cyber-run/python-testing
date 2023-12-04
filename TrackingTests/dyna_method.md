## Dyna method

Known info:
- 2 coordinate points; 1 of mirror axis and one of target
- absolute angle of mirror axis

Method:
- Set mirror rotation to central point; 45 degree incidence angle
- For this case, take the yaw angle of rigid body when set to this point
- Derive the vector of this angle traj from coordinate of mirror
- Use this as central reference for future calculations
- Get target coordinate and calculate vector to target coordiante from mirror coordinate
- Then find the angle between target and central reference vector
- then find the angle between target and mirror axis