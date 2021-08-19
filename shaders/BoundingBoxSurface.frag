#version 330

uniform vec4 u_color;
uniform vec3 u_cam_pos;

uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

uniform sampler3D u_density_tex;

uniform vec3 u_bbox_min;
uniform vec3 u_bbox_max;

uniform float u_cloud_scale;
uniform vec3 u_cloud_offset;
uniform float u_density_thresh;
uniform float u_density_mult;
uniform int u_density_samples;

in vec4 v_position;
in vec4 v_normal;

out vec4 out_color;

vec2 rayBoxDst(vec3 boundsMin, vec3 boundsMax, vec3 rayOrigin, vec3 invRaydir);


float sampleDensity( vec3 pos ) {
    vec3 xyz = pos * u_cloud_scale * 0.001 + u_cloud_offset * 0.01;
    float d = texture( u_density_tex, xyz ).x;

    return max( 0, d - u_density_thresh * 0.1 ) * u_density_mult;
}

void main() {
    vec3 wo = normalize( u_cam_pos - vec3( 2. * v_position ) );
    vec2 distances = rayBoxDst( u_bbox_min, u_bbox_max, u_cam_pos, -wo );
    float d_to_box = distances.x;
    float d_in_box = distances.y;

    float d = 0.f;
    float d_travd = 0.f;
    int n = 0;
    float step_size = d_in_box / u_density_samples;
    while ( d_travd < d_in_box ) {

        vec3 v = u_cam_pos + ( d_travd + d_to_box ) * -wo;

        d += sampleDensity( v ) * step_size;

        d_travd += step_size;
        n += 1;
    }

    d = exp( -( d ) );
    out_color = vec4( d, d, d, d );
}


/* reference: https://github.com/SebLague/Clouds/blob/master/Assets/Scripts/Clouds/Shaders/Clouds.shader */
vec2 rayBoxDst(vec3 boundsMin, vec3 boundsMax, vec3 rayOrigin, vec3 invRaydir) {
    // Adapted from: http://jcgt.org/published/0007/03/04/
    vec3 t0 = (boundsMin - rayOrigin) * invRaydir;
    vec3 t1 = (boundsMax - rayOrigin) * invRaydir;
    vec3 tmin = min(t0, t1);
    vec3 tmax = max(t0, t1);
    
    float dstA = max(max(tmin.x, tmin.y), tmin.z);
    float dstB = min(tmax.x, min(tmax.y, tmax.z));

    // CASE 1: ray intersects box from outside (0 <= dstA <= dstB)
    // dstA is dst to nearest intersection, dstB dst to far intersection

    // CASE 2: ray intersects box from inside (dstA < 0 < dstB)
    // dstA is the dst to intersection behind the ray, dstB is dst to forward intersection

    // CASE 3: ray misses box (dstA > dstB)

    float dstToBox = max(0, dstA);
    float dstInsideBox = max(0, dstB - dstToBox);
    return vec2(dstToBox, dstInsideBox);
}

