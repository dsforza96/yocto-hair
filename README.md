# Yocto/Hair: Tiny Path Tracer Extension to Shade Realistic-Looking Hairs

**Authors:** [Antonio Musolino](https://github.com/antoniomuso) and [Davide Sforza](https://github.com/dsforza96)

Yocto/Hair is a tiny extension of [Yocto/GL](https://github.com/xelatihy/yocto-gl) to shade realistic looking hairs. 
Our code follows the [pbrt](https://www.pbrt.org/hair.pdf) implementation.


## Geometry
We decided to use hair models from [Benedikt Bitterli](benedikt-bitterli.me/resources). We converted *.pbrt* scenes into Yocto/GL *.json* scenes format.
Because Yocto/GL doesn't support Bézier curves we approximated them into straight lines. We used two lines for each Bézier curve to render straight hairs and four lines to render curly hairs. We also store lines tangents and linear interpolated widths for each vertex.  To optimize the rendering performance we joined all the lines into one only *.ply* shape.
## Scattering Model Implementation
We followed straightforwardly the implementation presented in [pbrt](https://www.pbrt.org/hair.pdf). We implemented three new functions:
- `eval_hair_scattering(...)`: Given incoming and outgoing directions computes the corrisponding hair BRDF;
- `sample_hair_scattering(...)`: Given the outgoing direction sample an incoming direction according to BRDF;
- `eval_hair_scattering_pdf(...)`: Returns the PDF related to incoming and outgoing directions.

Since pbrt computations are made in local BRDF coordinate system, we built 
## Hair Material
## Examples
### Longitudinal Scattering
### Absortion in Fibers
### Aimuthal Scattering
### The Effect of Scales on Hair
## Tests
## Yocto/GL Files Modifications

## License
Our code is released under [MIT](LICENSE) license.
