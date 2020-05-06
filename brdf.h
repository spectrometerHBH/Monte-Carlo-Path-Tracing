#ifndef CG4__BRDF_H_
#define CG4__BRDF_H_

class BRDFMaterial {
 public:
  virtual bool isMirror() const = 0;
  virtual bool isLight() const = 0;
  virtual Ogre::Vector3 getBRDF() const = 0;
};

class DiffuseMaterial : public BRDFMaterial {
 public:
  Ogre::Vector3 diffuse;

  explicit DiffuseMaterial(const Ogre::Vector3& diffuse) : diffuse(diffuse) {}

  bool isMirror() const override {
    return false;
  }

  bool isLight() const override {
    return false;
  }

  Ogre::Vector3 getBRDF() const override {
    return diffuse / M_PI;
  }
};

class LightMaterial : public BRDFMaterial {
 public:
  Ogre::Vector3 emissive;

  explicit LightMaterial(const Ogre::Vector3& emissive) : emissive(emissive) {}

  bool isMirror() const override {
    return false;
  }

  bool isLight() const override {
    return true;
  }

  Ogre::Vector3 getBRDF() const override {
    return emissive;
  }
};

class MirrorMaterial : public BRDFMaterial {
 public:
  MirrorMaterial() = default;

  bool isMirror() const override {
    return true;
  }

  bool isLight() const override {
    return false;
  }

  Ogre::Vector3 getBRDF() const override {
    return Ogre::Vector3::ZERO;
  }
};

#endif //CG4__BRDF_H_
