#version 330

uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

uniform sampler3D u_density_tex;
uniform sampler3D u_noise;

uniform vec3 u_bbox_min;
uniform vec3 u_bbox_max;

/*
 * Cloud Parameters
 */
uniform float u_cloud_scale;
uniform vec3 u_cloud_offset;
uniform float u_density_thresh;
uniform float u_density_mult;
uniform int u_density_samples;

/*
 * Light Parameters
 */
uniform float u_lt_abs_sun;
uniform float u_lt_abs_cloud;
uniform float u_lt_darkness;
uniform vec4  u_lt_phase;

in vec4 v_position;
in vec3 v_origin;
in vec3 v_raydir;

out vec4 out_color;

/*
 * Scale a value v which was between lo and ho to be ln and hn
 */
float scale( float v, float lo, float ho, float ln, float hn ) {
    return ln + ( (v - lo) * (hn - ln) ) / (ho - lo);
}

/* Henyey-Greenstein helper function for
 * phase() function below
 * g <- [-1, 1]
 *   selects betwween back scattering,
 *   isotropic, and forward scattering
 * */
float hg(float a, float g) {
    float g2 = g * g;
    return ( 1.0 - g2 ) / ( 4.0 * 3.1415 * pow( 1.0 + g2 - 2.0 * g * a, 1.5) );
}

/* This function provides us with a proper rotation
 * towards the sun which will make clouds
 * appear brighter closer to the sun */
float phase( float a ) {
    float blend = 0.5;
    float hgBlend = hg( a, u_lt_phase.x * 0.01 ) * ( 1.0 - blend ) + hg( a, -u_lt_phase.y * 0.01 ) * blend;
    return u_lt_phase.z + hgBlend * u_lt_phase.w;
}

/* Sample provided textures and return a single
 * float number representing a shade of a pixel
 * on the quad at current position */
float sampleDensity( vec4 pos ) {
    // vec3 xyz = pos.xyz;
    // vec3 xyz = pos.xyz + u_cloud_offset * 0.1;
    vec3 xyz = pos.xyz * u_cloud_scale * 0.01 + u_cloud_offset * 0.1;
    vec4 tex = texture( u_noise, ( xyz.xyz + 0.5 ) );

    // Generate higher frequency worley using same texture
    float freq2 = texture( u_noise, 4 * ( xyz.xyz + 0.5 ) ).g;
    float freq3 = texture( u_noise, 16 * ( xyz.xyz + 0.5 ) ).g;

    // Fractional Brownion Motion to decay the higher freq noise
    float shape_noise = tex.g * (0.625) + freq2 * (0.25) + freq3 * (0.125);
    shape_noise = shape_noise - 1;
    shape_noise = scale( tex.r, shape_noise, 1, 0, 1 );

    // Don't sample close to edges of box
    float PI = 3.14;
    float xfilter = sin( PI * pos.x / u_bbox_max.x );
    xfilter *= xfilter;
    float yfilter = sin( PI * pos.y / u_bbox_max.y );
    yfilter *= yfilter;
    float zfilter = sin( PI * pos.z / u_bbox_max.z );
    zfilter *= zfilter;

    return max( 0, shape_noise - u_density_thresh * 0.1 ) * u_density_mult * 0.01 * xfilter * yfilter * zfilter;
}

/* Check for ray-box intersecation returning:
 * (-1, -1, -1)                     if no intersection
 * (time_of_entry, time_of_exit, 1) if intersection */
vec3 intersect( vec3 boxmin, vec3 boxmax, vec3 o, vec3 d ) {
  float tx1 = ( boxmin.x - o.x ) / d.x;
  float tx2 = ( boxmax.x - o.x ) / d.x;

  // y-axis
  float ty1 = ( boxmin.y - o.y ) / d.y;
  float ty2 = ( boxmax.y - o.y ) / d.y;

  // z-axis
  float tz1 = ( boxmin.z - o.z ) / d.z;
  float tz2 = ( boxmax.z - o.z ) / d.z;

  float txmin = min( tx1, tx2 );
  float txmax = max( tx1, tx2 );
  float tymin = min( ty1, ty2 );
  float tymax = max( ty1, ty2 );
  float tzmin = min( tz1, tz2 );
  float tzmax = max( tz1, tz2 );

  float tmin = max( max( txmin, tymin ), tzmin );
  float tmax = min( min( txmax, tymax), tzmax );

  if ( tmin > tmax || tmax < 0 ) { return vec3(-1, -1, -1); }

  return vec3( tmin, tmax, 1 );
}

/* Calculate how much light reaches
 * current position */
float lightmarch( vec3 pos ) {
  vec3 dirToLight = normalize(vec3(u_light_pos) - pos);                     // Vector pointing from Ray to Light Source
  float distInBox = intersect( u_bbox_min, u_bbox_max, pos, dirToLight ).y; // Distance from point to edge of the box pointing towards the sun

  float step_size = distInBox / u_density_samples;
  float totalDensity = 0;

  for (int step = 0; step < u_density_samples; step++) {
      pos += dirToLight * step_size;
      totalDensity += max(0, sampleDensity(vec4( pos, 1 )) * step_size);
  }

  float transmittance = exp(-totalDensity * u_lt_abs_sun);
  return u_lt_darkness + transmittance * (1. - u_lt_darkness * 0.001);
}

void main() {
    vec3 o = v_origin;
    vec3 d = v_raydir;

    /* Generate blue sky */
    // out_color = vec4( 0.39, 0.74, 0.99, 1 );
    float blue = scale( v_position.y, -1, 1, 0.4, 0.6 );
    out_color = vec4( vec3( 0.39, 0.74, 0.99 ) - blue, 1 );

    /* Check for ray-box intersection
     * returning distance to the box,
     * distance traveled in the box, and
     * "1" is placed in z-coordinate to indicate
     * a hit! */
    vec3 dists = intersect( u_bbox_min, u_bbox_max, o, d );

    /* z-coordinate is 1 therefore we have a positive
     * ray-box-intersecion --> let's draw on the quad */
    if ( dists.z == 1 ) {
      float d_to_box = dists.x;
      float d_in_box = dists.y - dists.x;

      float val = 0.f;
      float d_travd = 0.f;
      float step_size = d_in_box / u_density_samples;

      // Light marching variables:
      float transmittance = 1;
      float lightEnergy = 0;

      /* Step through the box sampling the density
       * and accumulating the result for output to
       * screen */
      vec3 test_pos = o + (d_travd + d_to_box) * d;
      while ( d_travd < d_in_box ) {
          vec3 v = o + ( d_travd + d_to_box ) * d;
          val += sampleDensity( vec4( v, 1 ) ) * step_size;

          /* If density is larger than 0:
           * We will lightmarch through the cloud
           * adding up the light for given ray */
          if (val > 0) {
              float light_transmittance = lightmarch(v);

              // Calculate Phase Value:
              float cos_theta = dot( v, normalize( u_light_pos ) );
              float phase_val = phase( cos_theta );

              lightEnergy += val * step_size * transmittance * light_transmittance * phase_val * ( v.y / u_bbox_max.y );
              transmittance *= exp(-val * step_size * u_lt_abs_cloud);
              if (transmittance < 0.01) { break ; }
          }
          d_travd += step_size;
      }

      // val = exp( -( 1 - val ) );

      /* Final out color mixing:
       * vec3 backgroundCol = tex2D(_MainTex,i.uv);
       * vec3 cloudCol = lightEnergy * _LightColor0;
       * vec3 col = backgroundCol * transmittance + cloudCol;
       * return float4(col,0); */

      vec3 cloudCol = lightEnergy * vec3( 1, 1, 1 );
      vec3 col = out_color.xyz * transmittance + cloudCol;

      out_color = vec4( col, 1 );
      // out_color = vec4( 0, 0, 0, val - test_light * 0.5);
    }
    /* z-coordinate is -1 therefore we have no
     * ray-box intersection and so we do not draw
     * anything to the screen! */
}

