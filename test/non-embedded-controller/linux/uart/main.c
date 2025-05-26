#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <math.h>
#include <poll.h> // For event-driven I/O

// Translational velocity desired in m/s
#define vd 0.075

// Gains
#define k1 0.19
#define k2 1.94
#define k3 0.90
#define k4 2.57

#define kp 0.08
#define kv 0.008

// Parameters, ranges and factors
#define tauM 0.3    // Max torque on wheel [Nm]
#define alphaM 1.4
#define omegaM 16.0
#define uM 11.0
#define uNM 11.0

#define Ts 0.01
#define ppr 12.0
#define pi_ 3.141593
#define pi_s2 1.570796
#define Ra 3.0
#define NR 44.0 // 58.0//15.5//34.014
#define R 0.033 // 0.035
#define km 0.0008 // 0.01186
#define Cz 0.0485
#define Mp 0.4580
#define b 0.09

#define calpha 0.145 // 0.2

#define deg_2_rad pi_ / 180
#define rad_2_deg 180 / pi_
#define accel_div_factor 16384.0
#define gyro_div_factor 131.0
#define accel_factor 1 / accel_div_factor
#define gyro_factor 1 / gyro_div_factor

#define c1 0.993

// FUNCTION PROTOTYPES
unsigned char v_to_pwm(float voltage);

unsigned char flagcom = 0, flagfile = 0, signo_sal, pwm, posr, posl, ang; // 8-bit
// PWM output to motors and sensors
unsigned char uWr, uWl, Sr, Sl;
signed char incr, incl;
// MPU values
signed short int Ax, Gy;
// Variables for complementary filter and angles
float Xa = 0, Yg = 0, alpha = 0;
float accelx = 0.15, angulox = 0.15, angulox_1 = 0, c2 = 1 - c1;
// Auxiliary variables
float t = 0, iTs = 1 / Ts, esc = pi_ / (2 * ppr * NR), escs = 127.0 / uM;
// Velocities
float omegar = 0, omegal = 0;
// Motor voltages
float ur = 0, ul = 0;
float v = 0, alpha_1 = 0, theta = 0, thetad = 0, thetatil = 0, taua, u;
float alphap = 0, vtil = 0, thetap = 0, taur = 0, taul = 0;
float Rasnkm = Ra / (NR * km), nkm = NR * km, intvtil = 0.0, v2tauM = 2 * tauM;

int main() {
    int fd; // File descriptor for the serial port
    struct termios tty; // Structure for serial port configuration
    FILE *fp;

    // Open file for logging data
    if ((fp = fopen("datos.txt", "w+")) == NULL) {
        perror("Error opening file");
        exit(EXIT_FAILURE);
    }

    // Open the serial port
    fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        perror("Error opening serial port");
        fclose(fp);
        exit(EXIT_FAILURE);
    }

    // Get current serial port configuration
    if (tcgetattr(fd, &tty) != 0) {
        perror("Error getting serial port attributes");
        close(fd);
        fclose(fp);
        exit(EXIT_FAILURE);
    }

    // Configure serial port for low latency
    cfsetospeed(&tty, B115200); // Output baud rate
    cfsetispeed(&tty, B115200); // Input baud rate

    tty.c_cflag &= ~PARENB; // No parity bit
    tty.c_cflag &= ~CSTOPB; // 1 stop bit
    tty.c_cflag &= ~CSIZE;  // Clear data size bits
    tty.c_cflag |= CS8;     // 8 bits per byte
    tty.c_cflag &= ~CRTSCTS; // Disable hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; // Enable receiver, ignore modem control lines

    tty.c_lflag &= ~ICANON; // Disable canonical mode (raw input)
    tty.c_lflag &= ~ECHO;   // Disable echo
    tty.c_lflag &= ~ECHOE;  // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG;   // Disable interpretation of INTR, QUIT and SUSP

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off software flow control
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (raw output)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

    tty.c_cc[VMIN] = 0;  // Read doesn't block
    tty.c_cc[VTIME] = 0; // No inter-character timer (0.0 second timeout)

    // Set the new configuration
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("Error setting serial port attributes");
        close(fd);
        fclose(fp);
        exit(EXIT_FAILURE);
    }

    // Flush any pending data
    tcflush(fd, TCIOFLUSH);

    char received_byte;
    ssize_t bytes_read;
    struct pollfd pfd;
    pfd.fd = fd;
    pfd.events = POLLIN; // Monitor for input events
    const unsigned char start_byte = 170;

    while (1) {
        // Use poll to wait for data with a timeout (e.g., 100ms)
        int poll_ret = poll(&pfd, 1, 100); // 100ms timeout
        if (poll_ret < 0) {
            perror("Poll error");
            break;
        } else if (poll_ret == 0) {
            // Timeout, no data received, continue loop
            continue;
        }

        // Data is available, try to read
        bytes_read = read(fd, &received_byte, 1);
        if (bytes_read < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // No data currently available, but it's non-blocking. This should ideally not happen
                // often if poll indicated data was ready.
                continue;
            } else {
                perror("Error reading from serial port");
                break;
            }
        } else if (bytes_read == 0) {
            // End of file, or device disconnected (rare for serial ports)
            continue;
        } else {
            // A byte was successfully read
            if (flagcom != 0)
                flagcom++;
            if ((received_byte == start_byte) && (flagcom == 0)) {
                flagcom = 1;
            }
            if (flagcom == 2) {
                posr = received_byte;
                incr = 127 - posr;
            }

            if (flagcom == 3) {
                posl = received_byte;
                incl = 127 - posl;
            }
            if (flagcom == 4) {
                Ax = received_byte;
                Ax = Ax << 8;
            }
            if (flagcom == 5) {
                Ax = Ax + received_byte;
            }
            if (flagcom == 6) {
                Gy = received_byte;
                Gy = Gy << 8;
            }
            if (flagcom == 7) {
                Gy = Gy + received_byte;
            }
            if (flagcom == 8) {
                Sr = received_byte;
            }
            if (flagcom == 9) {
                Sl = received_byte;
                theta = Sr - Sl;
                if (theta > 160)
                    theta = 160;
                if (theta < -160)
                    theta = -160;
                theta = -0.3 * theta / 160.0; // Tracking error in radians
                // Calculate right velocity in radians.
                omegar = incr * esc * iTs;
                // Calculate left velocity in radians.
                omegal = incl * esc * iTs;
                // MPU values are converted to float with their respective scales.
                Xa = -Ax * accel_factor; // inclination in range of -1 to 1
                Yg = Gy * gyro_factor;   // Yg in degrees per second
                Yg = deg_2_rad * Yg;     // Yg in rad per second
                // Complementary filter calculation
                accelx = Xa * pi_s2; // inclination in range of -pi/2 to pi/2 rad
                angulox = c1 * (angulox_1 + Yg * Ts) + c2 * accelx; // complementary filter equation
                angulox_1 = angulox; // storing previous value
                alpha = -angulox;
                alpha = alpha - calpha; // IMU alignment compensation
                if (alpha >= alphaM)
                    alpha = alphaM;
                if (alpha <= (-alphaM))
                    alpha = -alphaM;
                // Calculate translational velocity
                v = (omegar + omegal) * R / 2;

                thetap = (omegar - omegal) * R / (2 * b);

                thetatil = theta - thetad;          // theta tilde
                alphap = (alpha - alpha_1) * iTs; // alpha dot
                alpha_1 = alpha;
                vtil = v - vd; // v tilde
                if ((intvtil < v2tauM) && (intvtil > (-v2tauM))) // Integral of v tilde
                    intvtil = intvtil + Ts * vtil;
                else {
                    if (intvtil >= v2tauM)
                        intvtil = 0.95 * v2tauM;
                    if (intvtil <= (-v2tauM))
                        intvtil = -0.95 * v2tauM;
                }
                // Control efforts
                taua = (-kv * thetap - kp * thetatil) * 2 * b / R;
                u = k1 * alphap + k2 * alpha + k3 * vtil + k4 * intvtil;
                if (u >= v2tauM)
                    u = v2tauM;
                if (u <= (-v2tauM))
                    u = -v2tauM;

                // Torques per wheel
                taur = (taua + u) / 2.0; // right torque
                taul = (-taua + u) / 2.0; // left torque

                // Left wheel voltage
                ul = taul * Rasnkm + nkm * omegal;
                if (ul >= uNM)
                    ul = uNM;
                if (ul <= (-uNM))
                    ul = -uNM;
                uWl = v_to_pwm(ul);
                char send_byte_l = uWl;
                // Send left wheel command
                if (write(fd, &send_byte_l, 1) < 0) {
                    perror("Error writing to serial port for left wheel");
                }

                // Right wheel voltage
                ur = taur * Rasnkm + nkm * omegar;
                if (ur >= uNM)
                    ur = uNM;
                if (ur <= (-uNM))
                    ur = -uNM;
                uWr = v_to_pwm(ur);
                char send_byte_r = uWr;
                // Send right wheel command
                if (write(fd, &send_byte_r, 1) < 0) {
                    perror("Error writing to serial port for right wheel");
                }

                // Printing to console
                printf("%.2f\t%.2f\t%.2f\n", t, alpha, theta);
                // Write some data to the file
                fprintf(fp, "%.2f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n", t, alpha, theta, v, vd, omegal, omegar, ul, ur);
                t = t + Ts; // t+=Ts;
                flagcom = 0;
            }
        }
    }

    fclose(fp);
    close(fd);
    return 0;
}

unsigned char v_to_pwm(float voltage) {
    float pwmf;
    pwmf = escs * voltage;
    pwm = (unsigned char)fabs(pwmf);
    if (pwmf < 0)
        pwm = pwm + 128;
    return pwm;
}