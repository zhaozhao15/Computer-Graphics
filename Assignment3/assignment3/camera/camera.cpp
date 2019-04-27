#include "camera.h"
#include "../myrand.h"
namespace Orchid
{
	Camera::Camera
	(
		Vector3d position, Vector3d target, Vector3d up,
		int width, int height,
		double nearPalneDistance, double fov
	) :	_position{ position },
		_up{ up },
		_imageW{ width },
		_imageH{ height },
		_nearPlaneDistance{ nearPalneDistance }
	{
		_cameraVerticalFOV = fov *PI / 180.0f;
		_aspectRatio = (double)_imageW / _imageH;
		_cameraFwd = (target - _position).normalized();
		_cameraRight = _up.cross(_cameraFwd*(-1.0)).normalized();
		_cameraUp = _cameraRight.cross(_cameraFwd).normalized();///???
		_cameraHorizFOV = _cameraVerticalFOV *_aspectRatio;
		_windowTop = tan(_cameraVerticalFOV / 2.0f)*_nearPlaneDistance;
		_windowRight = tan(_cameraHorizFOV / 2.0f)*_nearPlaneDistance;
	}

	Ray Camera::get_ray(int x, int y, bool jitter, unsigned short * Xi)
	{
		double x_jitter;
		double y_jitter;
		double m_ratio = (double)_imageW/ _imageH;
		double m_x_spacing = (2.0 * m_ratio) / (double)_imageW;
		double m_y_spacing = (double)2.0 / (double)_imageH;
		// If jitter == true, jitter point for anti-aliasing
		if (jitter) {
			x_jitter = (myrand(Xi) * m_x_spacing) - m_x_spacing*0.5;
			y_jitter = (myrand(Xi) * m_y_spacing) - m_y_spacing*0.5;

		}
		else {
			x_jitter = 0;
			y_jitter = 0;
		}

		Vector3d pixel = _position + _cameraFwd * 2;
		pixel = pixel - _cameraRight * m_ratio + _cameraRight * ((x * 2 * m_ratio)/ _imageW) + x_jitter;
		pixel = pixel + _cameraUp - _cameraUp * ((y * 2.0)/ _imageH + y_jitter);

		return Ray(_position, (pixel - _position).normalized());

	}
}
