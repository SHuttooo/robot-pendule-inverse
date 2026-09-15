# -*- coding: utf-8 -*-
"""
Ton firmware, transcrit.

Portage direct de robot_balancier.ino : memes gains, memes unites (degres et
pas/s), meme cascade. Rien n est "adapte a la simu".

  boucle interne  200 Hz   accel = Kp*err + Ki*integ + Kd*gyroFilt   [pas/s2]
                           wheel_sps += accel*dt   <- l integrateur EST la commande
  boucle externe   40 Hz   target_angle = angleOffset - (Kp_spd*e_v + Ki_spd*e_p)

Module partage par 03_pid.py, 04_pousser.py et rendu.py.
Seul ecart avec le vrai robot : ici la verticale vaut 0 deg. Sur le tien elle
vaut -91,4 deg parce que le MPU est monte couche.
"""
import math

STEPS_PER_REV = 1600.0
RAD_PAR_PAS = 2 * math.pi / STEPS_PER_REV


class Firmware:
    """Transcription de robot_balancier.ino."""

    # ------- reglage final du carnet, enregistre en flash
    Kp_a, Ki_a, Kd_a = 4500.0, 40.0, 600.0     # pas/s2 par deg, par deg.s, par deg/s
    Kp_spd, Ki_spd = 0.002, 0.0008             # deg par pas/s, deg par pas
    MAX_SPS, MAX_ACCEL = 1600.0, 25000.0
    TAU_SPEED = 0.40
    # ------- constantes non reglables, dans le source
    TAU_COMP, TAU_GYRO = 1.0, 0.012
    ACCEL_DERATE = 0.60
    MAX_TILT_CORR, MAX_POS_ERR = 6.0, 30000.0
    OUTER_FADE_LO, OUTER_FADE_HI = 3.0, 6.0
    angleOffset = 0.0                          # -91,4 sur le vrai robot

    def __init__(self, dual=True):
        self.dual = dual
        self.pitch = 0.0
        self.gyroRate = self.gyroFilt = 0.0
        self.wheel_sps = self.angleIntegral = 0.0
        self.filtered_speed = self.target_pos = 0.0
        self.speed_angle_corr = 0.0
        self.target_angle = self.angleOffset

    # ---------------------------------------------------------------- l IMU
    def imu(self, gyro, acc, dt):
        """Filtre complementaire, ligne pour ligne comme le .ino.
        Sur le robot : atan2(ay, az), car le MPU est couche.
        Ici le site est aligne sur le chassis, donc atan2(-ax, az)."""
        self.gyroRate = math.degrees(gyro[1])
        gyroPitch = self.pitch + self.gyroRate * dt
        magG = math.hypot(acc[0], acc[2]) / 9.81
        if 0.6 < magG < 1.4:
            accPitch = math.degrees(math.atan2(-acc[0], acc[2]))
            alpha = self.TAU_COMP / (self.TAU_COMP + dt)
            self.pitch = alpha * gyroPitch + (1 - alpha) * accPitch
        else:
            self.pitch = gyroPitch
        # lissage dedie au terme D : sans lui les vibrations dominent la sortie
        self.gyroFilt += (self.gyroRate - self.gyroFilt) * (dt / (self.TAU_GYRO + dt))

    # ------------------------------------------------------- boucle interne
    def inner(self, dt):
        err = self.pitch - self.target_angle

        self.angleIntegral = min(20.0, max(-20.0, self.angleIntegral + err * dt))
        accel = self.Kp_a * err + self.Ki_a * self.angleIntegral + self.Kd_a * self.gyroFilt

        # limite d acceleration deratee selon la vitesse (courbe couple-vitesse)
        lim = self.MAX_ACCEL * (1 - self.ACCEL_DERATE * abs(self.wheel_sps) / self.MAX_SPS)
        lim = max(lim, self.MAX_ACCEL * (1 - self.ACCEL_DERATE))
        accel = min(lim, max(-lim, accel))

        self.wheel_sps += accel * dt

        # anti-emballement : sur saturation de vitesse on gele l integrale d angle
        if self.wheel_sps > self.MAX_SPS:
            self.wheel_sps = self.MAX_SPS
            if err > 0:
                self.angleIntegral -= err * dt
        elif self.wheel_sps < -self.MAX_SPS:
            self.wheel_sps = -self.MAX_SPS
            if err < 0:
                self.angleIntegral -= err * dt
        return err

    # ------------------------------------------------------- boucle externe
    def outer(self, dt, pos_pas):
        if not self.dual:
            self.target_angle = self.angleOffset
            return
        k = dt / (self.TAU_SPEED + dt)
        self.filtered_speed += (self.wheel_sps - self.filtered_speed) * k

        pos_err = min(self.MAX_POS_ERR, max(-self.MAX_POS_ERR, pos_pas - self.target_pos))
        speed_err = self.filtered_speed                   # cible de vitesse nulle

        # monter la cible fait accelerer en positif -> pour freiner, on BAISSE
        corr = self.Kp_spd * speed_err + self.Ki_spd * pos_err
        corr = min(self.MAX_TILT_CORR, max(-self.MAX_TILT_CORR, corr))

        # fondu : pendant une grosse perturbation la boucle externe s efface,
        # sinon le robot vise un angle decale au lieu de son vrai equilibre
        distur = abs(self.pitch - self.angleOffset)
        w = (self.OUTER_FADE_HI - distur) / (self.OUTER_FADE_HI - self.OUTER_FADE_LO)
        corr *= min(1.0, max(0.0, w))

        self.speed_angle_corr = corr
        self.target_angle = self.angleOffset - corr
