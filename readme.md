# Yocto/Examples: Examples of Usage of Yocto/GL

See [Yocto/GL](https://github.com/xelatihy/yocto-gl/) for the documentation of the main library.

## Color grading

The function `grade_image()` applies the following corrections.

- **Tone mapping** applies three successive corrections:
  - expoure compensation: `c = c * 2^exposure`
  - filmic correction: `c *= 0.6; c = (c^2 * 2.51 + c * 0.03) / (c^2 * 2.43 + c * 0.59 + 0.14)`
    that is a fit of the ACES tonemapper
  - srgb color space: `c = c ^ 1/2.2`
  - clamp result: `c = clamp(c, 0, 1)` con `clamp(c, m, M) = max(min(c, M), m)`
- **Color tint**: `c = c * tint`
- **Saturation**: `c = g + (c - g) * (saturation * 2)` con
  - `g = (c.r + c.g + c.b)/3`
- **Contrast**: `c = gain(c, 1 - contrast)` con
  - `gain(a, b) = (a < 0.5) ? bias(a * 2, b) / 2 : bias(a * 2 - 1, 1 - b) / 2 + 0.5`
  - `bias(a, b) = a / ((1 / b - 2) * (1 - a) + 1)`
- **Vignette**: `c = c * (1 - smoothstep(vr, 2 * vr, r))` con
  - `vr = 1 - vignette`
  - `r = length(ij - size/2) / length(size/2)`
  - `size` is the image dimension
  - `ij` le coordinate del pixel
  - `smoothstep(a, b, u) = t * t * (3 - 2 * t)` con
    `t = clamp((u - a) / (b - a), 0, 1)`
- **Film grain**: `c = c + (rand1f(rng) - 0.5) * grain`
  - to generate random numbers, use the `rng_state` generator, created with
    `make_rng()` and used with `rand1f()`
- **Mosaic Effect**: `c[i, j] = c[i - i % mosaic, j - j % mosaic]`
- **Grid Effect**: `c[i, j] = (0 == i % grid || 0 == j % grid) ? 0.5 * c : c`

The corrections are applied one after the other in the order given.

Other example manipulation that could be done are:

- **ShaderToy Filters**: example filters from [ShaderToy](https://www.shadertoy.com)
- **The Book Of Shaders Filters**: example filters from
  [The Book Of Shaders](https://thebookofshaders.com)
- **Instagram or TikTok Filter**: descriptions found online
- **Gaussian Blur**: for example from [Gaussian blur](https://en.wikipedia.org/wiki/Gaussian_blur)
  - with a possible implementation from [qui](https://blog.demofox.org/2015/08/19/gaussian-blur/)
- **Stippling**: [Wieghted Voronoi Stippling](https://mrl.cs.nyu.edu/~ajsecord/npar2002/npar2002_ajsecord_preprint.pdf)
  - with a possible implementation from [qui](https://maxhalford.github.io/blog/halftoning-2/)
- **Stippling2**: [Weighted Linde–Buzo–Gray Stippling](http://graphics.uni-konstanz.de/publikationen/Deussen2017LindeBuzoGray/WeightedLindeBuzoGrayStippling_authorversion.pdf)
  - several implementations on Github
- **Cloning**: [PatchMatch](https://gfx.cs.princeton.edu/pubs/Barnes_2009_PAR/index.php)
  - several implementation online Github
- **Clarity**: [Local Laplacian Filters](https://people.csail.mit.edu/sparis/publi/2011/siggraph/Paris_11_Local_Laplacian_Filters.pdf)
  - several implementations on Github
