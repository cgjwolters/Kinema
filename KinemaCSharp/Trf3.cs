public struct Trf3
{
  internal double[,] m = new double [3,4];

  public bool isDerivative;

  public void init()
  {
    m[0,0] = 1.0; m[0,1] = 0.0; m[0,2] = 0.0;
    m[1,0] = 0.0; m[1,1] = 1.0; m[1,2] = 0.0;

    isDerivative = false;
  }

  public void zero()                           // All elements to zero
  {
    m[0,0] = 0.0; m[0,1] = 0.0; m[0,2] = 0.0;
    m[1,0] = 0.0; m[1,1] = 0.0; m[1,2] = 0.0;
  
    isDerivative = false;
  }

  public Trf3() { init(); }                      // Initialize to unitmat
  public Trf3(double m00, double m01, double m02, double m03,
              double m10, double m11, double m12, double m13,
              double m20, double m21, double m22, double m23)
  {
    m[0,0] = m00; m[0,1] = m01; m[0,2] = m02;
    m[1,0] = m10; m[1,1] = m11; m[1,2] = m12;

    isDerivative = false;
  }


}

Trf3(Vec3 org, Vec3 zDir, Vec3&xDir)
{ 

}

double determinant() const;

double scaleX() const;
double scaleY() const;
double scaleZ() const;

bool mirrors() const { return determinant() < 0.0; } // Does it mirror?

    void mirror(const Vec3& org, const Vec3& mirrorAxis); // Mirror along
                                                          // axis

bool invert() { return invertInto(*this); } // Invert transform
bool invertInto(Trf3& invMat) const;        // Invert transform

Trf3 & operator = (const Trf3& mt);          // Assignment

Trf3 & operator += (const Trf3& mt);         // Add mt
Trf3 & operator -= (const Trf3& mt);         // Subtract mt
Trf3 & operator *= (double fact);            // Scaling

Trf3 & operator *= (const Trf3& mt);         // Post multiply with mt
Trf3 & preMultWith(const Trf3& mt);          // Pre multiply with mt

Vec3 operator *(const Vec3& p) const;       // Transform Vec3

double operator()(int ix, int iy) const;   // Return matrix element
double& operator()(int ix, int iy);         // Return matrix el. ref.

