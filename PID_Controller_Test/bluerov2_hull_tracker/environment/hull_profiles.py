#!/usr/bin/env python3
import numpy as np

class HullProfiles:
    """Generator for various ship hull profiles"""
    
    @staticmethod
    def wigley_hull(length=10.0, beam=1.5, draft=0.75, num_points=100):
        """Parabolic Wigley hull profile"""
        x = np.linspace(0, length, num_points)
        y = (beam/2) * (1 - (2*x/length - 1)**2) * (1 - (0.5/draft)**2)
        return np.column_stack((x, y))
    
    @staticmethod
    def tanker_hull(length=10.0, beam=2.0, draft=1.0, num_points=100):
        """Tanker hull with flatter bottom"""
        x = np.linspace(0, length, num_points)
        
        # Bow section (0-30%)
        bow_idx = x <= 0.3*length
        y_bow = draft * np.sin(np.pi * x[bow_idx] / (0.6*length))
        
        # Midship section (30-70%)
        mid_idx = (x > 0.3*length) & (x <= 0.7*length)
        y_mid = draft * np.ones(np.sum(mid_idx))
        
        # Stern section (70-100%)
        stern_idx = x > 0.7*length
        y_stern = draft * np.cos(np.pi * (x[stern_idx] - 0.7*length) / (0.6*length))
        
        y = np.concatenate((y_bow, y_mid, y_stern))
        y = (beam/2) * y / np.max(y)
        
        return np.column_stack((x, y))
    
    @staticmethod
    def destroyer_hull(length=10.0, beam=1.2, draft=0.6, num_points=100):
        """Narrow destroyer-type hull"""
        x = np.linspace(0, length, num_points)
        
        # Bow wave profile
        bow_wave = 0.2 * np.sin(2*np.pi*x/(0.4*length))
        
        # Base shape
        y = draft * np.exp(-5*(x/length - 0.5)**2)
        y += bow_wave * np.exp(-10*(x/length - 0.2)**2)
        
        # Stern taper
        stern_mask = x > 0.8*length
        y[stern_mask] *= np.linspace(1, 0.2, np.sum(stern_mask))
        
        return np.column_stack((x, (beam/2)*y/np.max(y)))