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

import numpy as np

__author__ = 'varribas'


class MedianSmoother():
    """ Median filter of input data
    """
    def __init__(self, H=5):
        self.H=H
        self.history = []
        self.median = None

    def feed(self, value):
        self.history += [value]
        h = len(self.history)
        if h > self.H:
            self.history = self.history[h-self.H:h]

        self.median = np.median(self.history)
        return self.median

    def value(self):
        return self.median


class MeanSmoother:
    """ Weighted mean filter of input data
    """
    def __init__(self, alpha=0.1):
        self.alpha = alpha
        self.value = 0

    def feed(self, value):
        self.value = self.alpha*value + (1-self.alpha)*self.value

        return self.value

    def value(self):
        return self.value
