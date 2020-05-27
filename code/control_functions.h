
//Control functions available for use in main.c

double* P(double* angle);
double* PI(double* angle, double* dt);
double* PID(double* angle, double* dt);
double* LQR(double* x, double* xdot, double* theta, double* thetadot);