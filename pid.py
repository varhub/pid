#
#  This file is part of pid control for python
#  https://github.com/varhub/pid.git
#
#  Copyright (C) 2016 Victor Arribas
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see http://www.gnu.org/licenses/.
#  Authors :
#       Victor Arribas <v.arribas.urjc@gmail.com>
#

__author__ = 'varribas'

Inf = float('Inf')


class PD:
    """ PD control
    """
    def __init__(self, reference=0, P=0, D=0):
        self.reference = reference
        self._last_value = reference

        self.P = P
        self.D = D

        self._feedback = 0

        self._errP = 0
        self._errD = 0

    def feedback(self, value=None):
        # idempotency
        if value is None:
            return self._feedback

        # error calculation
        self._errP = value - self.reference
        self._errD = self._errP - self._last_value

        # update
        self._last_value = value;

        # compute
        self._feedback = self.P * self._errP + self.D * self._errD
        return self._feedback

    def weight(self):
        return self.P + self.D

    def reset(self):
        self._last_value = self.reference
        self._errP = 0
        self._errD = 0


class PID(PD):
    """ Classical PID control from electronics with saturated Integral part
    """
    def __init__(self, reference=0, P=0, D=0, I=0, Imin=-Inf, Imax=Inf):
        PD.__init__(self, reference, P,D)

        self.I = I
        self.Imin = Imin
        self.Imax = Imax
        self._errI = 0

    def feedback(self, value=None):
        # idempotency
        if value is None:
            return self._feedback

        # error calculation
        _errP = value - self.reference
        self._errI += _errP

        if self._errI > self.Imax:
            self._errI = self.Imax
        if self._errI < self.Imin:
            self._errI = self.Imin

        self._feedback = PD.feedback(self,value) + self.I * self._errI
        return self._feedback

    def weight(self):
        return self.P + self.D + self.I

    def reset(self):
        PD.reset(self)
        self._errI = 0


class PIDh(PD):
    """ PID control with windowed Integration. Remember errors from last *Ilen* iterations
    """
    def __init__(self, reference=0, P=0, D=0, I=0, Ilen=10):
        PD.__init__(self, reference, P,D)

        self.I = I
        self.Ilen = Ilen
        self._error_history = []
        self._errI = 0

    def feedback(self, value=None):
        # idempotency
        if value is None:
            return self._feedback

        # error calculation
        _errP = value - self.reference

        self._error_history += [_errP]
        n = len(self._error_history)
        if n > self.Ilen:
            self._error_history = self._error_history[-self.Ilen:]
        self._errI = sum(self._error_history)

        self._feedback = PD.feedback(self,value) + self.I * self._errI
        return self._feedback

    def weight(self):
        return self.P + self.D + self.I

    def reset(self):
        PD.reset(self)
        self._errI = 0
        self._error_history = []
