#include <iostream>
#include <random>
#include "Ogre.h"
#include "OgreApplicationContext.h"
#include "bvh.h"
#include "brdf.h"

typedef unsigned char BYTE;

class RandomClass {
 public:
  std::random_device rd;
  std::mt19937 mt;
  std::uniform_real_distribution<double> dist;

  RandomClass() : rd{}, mt{rd()}, dist{0.0, 1.0} {}
  explicit RandomClass(double l, double r) : rd{}, mt{rd()}, dist(l, r) {}

  double rand() {
    return dist(mt);
  }
} ThetaRandomGen(0, M_PI / 2), PhiRandomGen(0, 2 * M_PI), LightRandom(-10, 10);

class BezierApp : public OgreBites::ApplicationContext {
 public:
  BezierApp() : OgreBites::ApplicationContext("OgreTutorialApp") {}
  void setup() override;
  void retrieveTrianglesFromEntity(const Ogre::Entity* entity, int source);
  void addOneEntity(BVHNode*& root, Ogre::Entity* entity, int source);
  Ogre::Vector3 pathTracing(Ogre::Ray ray, Ogre::Vector3 normal, int source, int bounce);
  TriangleTestResult intersect(Ogre::Ray ray);

  std::vector<Triangle> triangles;
  std::vector<BVHNode*> BVHroots;
  std::vector<BRDFMaterial*> Materials;
};

float clamp(Ogre::Real x) {
  if (x < 0) return 0;
  if (x > 1) return 1;
  return x;
}

float nonNega(Ogre::Real x) {
  if (x < 0) return 0;
  else return x;
}

void BezierApp::retrieveTrianglesFromEntity(const Ogre::Entity* entity, int source) {
  // calculate parent Pos/Orientation/Scale
  Ogre::SceneNode* parentNode = entity->getParentSceneNode();
  Ogre::Vector3 parentPos, parentScale;
  Ogre::Quaternion parentOrientation;
  if (parentNode != nullptr) {
    parentPos = parentNode->_getDerivedPosition();
    parentOrientation = parentNode->_getDerivedOrientation();
    parentScale = parentNode->_getDerivedScale();
  } else {
    parentPos = Ogre::Vector3::ZERO;
    parentOrientation = Ogre::Quaternion();
    parentScale = Ogre::Vector3::UNIT_SCALE;
  }
  Ogre::MeshPtr mesh = entity->getMesh();

  for (uint16_t i = 0; i < mesh->getNumSubMeshes(); i++) {
    Ogre::SubMesh* subMesh = mesh->getSubMesh(i);
    // ignore points and lines
    if ((subMesh->operationType == Ogre::RenderOperation::OperationType::OT_LINE_STRIP)
        || (subMesh->operationType == Ogre::RenderOperation::OperationType::OT_LINE_LIST)
        || (subMesh->operationType == Ogre::RenderOperation::OperationType::OT_LINE_STRIP)
        )
      continue;
    //-- prepare access to vertex buffer --
    Ogre::VertexData* vertexData;
    if (subMesh->useSharedVertices) {
      vertexData = mesh->sharedVertexData;
    } else {
      vertexData = subMesh->vertexData;
    }
    const Ogre::VertexElement* posEl =
        vertexData->vertexDeclaration->findElementBySemantic(Ogre::VertexElementSemantic::VES_POSITION);
    const Ogre::VertexElement* normalEl =
        vertexData->vertexDeclaration->findElementBySemantic(Ogre::VertexElementSemantic::VES_NORMAL);
    Ogre::HardwareVertexBufferSharedPtr
        vBuff = vertexData->vertexBufferBinding->getBuffer(posEl->getSource());
    uint32_t vertexSize = vBuff->getVertexSize();
    // save pointer to first vertex
    BYTE* pVertex = (BYTE*) vBuff->lock(Ogre::HardwareBuffer::LockOptions::HBL_READ_ONLY);
    BYTE* pStartVertex = pVertex;
    float* pReal, * nReal;  // pointer to grab position values
    //-- prepare access to index buffer --
    Ogre::HardwareIndexBufferSharedPtr iBuff = subMesh->indexData->indexBuffer;
    // Null can happen for meshes without triangles (e.g. a point cloud)
    if (iBuff == nullptr)
      continue;

    uint* pLong = (uint*) iBuff->lock(Ogre::HardwareBuffer::LockOptions::HBL_READ_ONLY);
    ushort* pShort = (ushort*) pLong;
    uint64_t index;
    //-- loop for each position by index --
    // different handling for operation types
    Ogre::Vector3 clampPos1, clampPos2, normal1, normal2;
    switch (subMesh->operationType) {
      //-- triangle LIST --
      case Ogre::RenderOperation::OperationType::OT_TRIANGLE_LIST: {
        for (uint32_t k = 0; k < subMesh->indexData->indexCount; k++) {
          // read index value
          if (iBuff->getType()
              == Ogre::HardwareIndexBuffer::IndexType::IT_32BIT)  // if 32bit indexes
            index = (uint64_t) pLong[k];
          else
            index = (uint64_t) pShort[k];
          // read referenced vertex
          pVertex = pStartVertex + (vertexSize * index);         // calculate pointer
          posEl->baseVertexPointerToElement(pVertex, &pReal);     // read vertex
          normalEl->baseVertexPointerToElement(pVertex, &nReal);
          Ogre::Vector3 vertexPos(pReal[0], pReal[1], pReal[2]); // read position values
          Ogre::Vector3 normal(nReal[0], nReal[1], nReal[2]);
          // apply world transformations
          if (parentNode != nullptr) {
            vertexPos = (parentOrientation * (vertexPos * parentScale)) + parentPos;
            normal = (parentOrientation * normal);
          }
          auto modulo = k % 3;
          if (modulo == 0) {
            clampPos1 = vertexPos;  // first corner of triangle
            normal1 = normal;
          } else if (modulo == 1) {
            clampPos2 = vertexPos;  // second corner of triangle
            normal2 = normal;
          } else {
            // triangle get
            this->triangles.emplace_back(clampPos1,
                                         clampPos2,
                                         vertexPos,
                                         (normal + normal1 + normal2).normalisedCopy(),
                                         source);
          }
        }
        break;
      }
        //-- triangle STRIP --
      case Ogre::RenderOperation::OperationType::OT_TRIANGLE_STRIP: {
        for (int k = 0; k < subMesh->indexData->indexCount; k++) {
          // read index value
          if (iBuff->getType()
              == Ogre::HardwareIndexBuffer::IndexType::IT_32BIT)  // if 32bit indexes
            index = (uint64_t) pLong[k];
          else
            index = (uint64_t) pShort[k];
          // read referenced vertex
          pVertex = pStartVertex + (vBuff->getVertexSize() * index);   // calculate pointer
          posEl->baseVertexPointerToElement(pVertex, &pReal);     // read vertex
          Ogre::Vector3 vertexPos(pReal[0], pReal[1], pReal[2]); // read position values
          normalEl->baseVertexPointerToElement(pVertex, &nReal);
          Ogre::Vector3 normal(nReal[0], nReal[1], nReal[2]);
          // apply world transformations
          if (parentNode != nullptr) {
            vertexPos = (parentOrientation * (vertexPos * parentScale)) + parentPos;
            normal = (parentOrientation * normal);
          }
          if (k >= 2) {
            this->triangles.emplace_back(clampPos1,
                                         clampPos2,
                                         vertexPos,
                                         (normal + normal1 + normal2).normalisedCopy(),
                                         source);
            // save 2 positions for next triangle
            clampPos1 = clampPos2;
            clampPos2 = vertexPos;
            normal1 = normal2;
            normal2 = normal;
          } else if (k == 0) {
            clampPos1 = vertexPos;
            normal1 = normal;
          } else {
            clampPos2 = vertexPos;
            normal2 = normal;
          }
        }
        break;
      }
        //-- triangle FAN --
      case Ogre::RenderOperation::OperationType::OT_TRIANGLE_FAN: {
        for (int k = 0; k < subMesh->indexData->indexCount; k++) {
          // read index value
          if (iBuff->getType()
              == Ogre::HardwareIndexBuffer::IndexType::IT_32BIT)  // if 32bit indexes
            index = (uint64_t) pLong[k];
          else
            index = (uint64_t) pShort[k];
          // read referenced vertex
          pVertex = pStartVertex + (vBuff->getVertexSize() * index);   // calculate pointer
          posEl->baseVertexPointerToElement(pVertex, &pReal);     // read vertex
          Ogre::Vector3 vertexPos(pReal[0], pReal[1], pReal[2]); // read position values
          normalEl->baseVertexPointerToElement(pVertex, &nReal);
          Ogre::Vector3 normal(nReal[0], nReal[1], nReal[2]);
          // apply world transformations
          if (parentNode != nullptr) {
            vertexPos = (parentOrientation * (vertexPos * parentScale)) + parentPos;
            normal = parentOrientation * vertexPos;
          }
          if (k >= 2) {
            this->triangles.emplace_back(clampPos1,
                                         clampPos2,
                                         vertexPos,
                                         (normal + normal1 + normal2).normalisedCopy(),
                                         source);
            // save position for next triangle
            clampPos2 = vertexPos;
            normal2 = normal;
          } else if (k == 0) {
            clampPos1 = vertexPos;
            normal1 = normal;
          } else {
            clampPos2 = vertexPos;
            normal2 = normal;
          }
        }
        break;
      }
    } // switch
    iBuff->unlock();
    vBuff->unlock();
  } // for
}

void BezierApp::addOneEntity(BVHNode*& BVHroot, Ogre::Entity* entity, int source) {
  this->triangles.clear();
  this->retrieveTrianglesFromEntity(entity, source);
  BVHroot = nullptr;
  build(BVHroot, this->triangles, 0, this->triangles.size() - 1);
}

Ogre::Vector3 BezierApp::pathTracing(Ogre::Ray rayOut,
                                     Ogre::Vector3 normal,
                                     int source,
                                     int bounce) {
  if (bounce == 0) return Ogre::Vector3::ZERO;
  if (this->Materials[source]->isLight()) {
    // emit
    return this->Materials[source]->getBRDF();
  } else if (this->Materials[source]->isMirror()) {
    // mirror
    Ogre::Vector3
        wi = 2 * (rayOut.getDirection().dotProduct(normal)) * normal - rayOut.getDirection();
    Ogre::Ray reflect(rayOut.getOrigin(), wi);
    TriangleTestResult result = intersect(reflect);
    if (result.hit) {
      return pathTracing(Ogre::Ray(reflect.getPoint(result.t), -wi),
                         result.normal,
                         result.source,
                         bounce - 1);
    } else {
      return Ogre::Vector3::ZERO;
    }
  } else {
    Ogre::Vector3 L_dir(0, 0, 0);
    Ogre::Vector3 wLight = (Ogre::Vector3(LightRandom.rand(), 74, LightRandom.rand())
        - rayOut.getOrigin()).normalisedCopy();
    if (wLight.dotProduct(normal) > 0) {
      Ogre::Ray rayLight(rayOut.getOrigin(), wLight);
      TriangleTestResult resultLight = intersect(rayLight);
      if (resultLight.hit && this->Materials[resultLight.source]->isLight()) {
        L_dir = this->Materials[resultLight.source]->getBRDF()
            * this->Materials[source]->getBRDF()
            * 20 * 20 / resultLight.t / resultLight.t
            * wLight.dotProduct(normal)
            * nonNega((-wLight).dotProduct(resultLight.normal));
      } else {
        L_dir = Ogre::Vector3::ZERO;
      }
    } else {
      L_dir = Ogre::Vector3::ZERO;
    }
    // reflections
    double theta = ThetaRandomGen.rand(), phi = PhiRandomGen.rand();
    Ogre::Vector3 wi(sin(theta) * sin(phi), cos(theta), sin(theta) * cos(phi));
    wi = (Ogre::Vector3::UNIT_Y.getRotationTo(normal)) * wi;
    wi.normalise();
    Ogre::Ray rayIn(rayOut.getOrigin(), wi);
    TriangleTestResult result = intersect(rayIn);
    if (result.hit && !this->Materials[result.source]->isLight()) {
      return L_dir + pathTracing(Ogre::Ray(rayIn.getPoint(result.t), -wi),
                                 result.normal,
                                 result.source,
                                 bounce - 1)
          * this->Materials[source]->getBRDF()
          * wi.dotProduct(normal) * 2 * M_PI;
    } else {
      return L_dir + Ogre::Vector3::ZERO;
    }
  }
}

TriangleTestResult BezierApp::intersect(Ogre::Ray ray) {
  TriangleTestResult res(false, 0, Ogre::Vector3::ZERO, -1);
  for (const auto* BVHroot : this->BVHroots)
    res = mergeTriangleTestResult(res, BVHintersect(BVHroot, ray));
  return res;
}

void BezierApp::setup() {
  // do not forget to call the base first
  OgreBites::ApplicationContext::setup();

  // get a pointer to the already created root
  Ogre::Root* root = getRoot();
  Ogre::SceneManager* scnMgr =
      root->createSceneManager(Ogre::DefaultSceneManagerFactory::FACTORY_TYPE_NAME, "MainScene");

  // register our scene with the RTSS
  Ogre::RTShader::ShaderGenerator::getSingletonPtr()->addSceneManager(scnMgr);

  // without light we would just get a black screen
  Ogre::Light* light_ogre = scnMgr->createLight("MainLight");
  Ogre::SceneNode* lightNode = scnMgr->getRootSceneNode()->createChildSceneNode();
  lightNode->setPosition(0, 70, 0);
  lightNode->attachObject(light_ogre);

  // also need to tell where we are
  Ogre::SceneNode* camNode = scnMgr->getRootSceneNode()->createChildSceneNode();
  camNode->setPosition(0, 0, 300);
  camNode->lookAt(Ogre::Vector3(0, 0, -1), Ogre::Node::TS_PARENT);

  // create the camera
  Ogre::Camera* camera = scnMgr->createCamera("myCam");
  camera->setNearClipDistance(5); // specific to this sample
  camera->setAutoAspectRatio(true);
  camNode->attachObject(camera);
  // and tell it to render into the main window
  getRenderWindow()->addViewport(camera);

  Ogre::Plane plane1(Ogre::Vector3::UNIT_Z,0);
  Ogre::Plane plane2(Ogre::Vector3::UNIT_X,-75);
  Ogre::Plane plane3(Ogre::Vector3::UNIT_Y,-75);
  Ogre::MeshManager::getSingleton().createPlane("wall1", Ogre::RGN_DEFAULT,
                                                plane1, 150, 150,
                                                20, 20, true,
                                                1, 1, 1,
                                                Ogre::Vector3::UNIT_X);
  Ogre::MeshManager::getSingleton().createPlane("wall2", Ogre::RGN_DEFAULT,
                                                plane2, 150, 150,
                                                20, 20, true,
                                                1, 1, 1,
                                                Ogre::Vector3::UNIT_Y);
  Ogre::MeshManager::getSingleton().createPlane("wall3", Ogre::RGN_DEFAULT,
                                                plane3, 150, 150,
                                                20, 20, true,
                                                1, 1, 1,
                                                Ogre::Vector3::UNIT_Z);
  Ogre::MeshManager::getSingleton().createPlane("light", Ogre::RGN_DEFAULT,
                                                plane3, 20, 20,
                                                200, 200, true,
                                                1, 1, 1,
                                                Ogre::Vector3::UNIT_Z);

  Ogre::Entity* wall1 = scnMgr->createEntity("wall1");
  wall1->setMaterialName("Wall");
  Ogre::Entity* wall2 = scnMgr->createEntity("wall2");
  wall2->setMaterialName("LeftWall");
  Ogre::Entity* wall3 = scnMgr->createEntity("wall2");
  wall3->setMaterialName("RightWall");
  Ogre::Entity* wall4 = scnMgr->createEntity("wall3");
  wall4->setMaterialName("Wall");
  Ogre::Entity* wall5 = scnMgr->createEntity("wall3");
  wall5->setMaterialName("Wall");
  Ogre::Entity* box1 = scnMgr->createEntity("cube.mesh");
  box1->setMaterialName("Box");
  Ogre::Entity* light = scnMgr->createEntity("light");
  Ogre::Entity* box2 = scnMgr->createEntity("cube.mesh");
  box2->setMaterialName("Box");

  scnMgr->getRootSceneNode()->createChildSceneNode()->attachObject(wall1);
  wall1->getParentSceneNode()->translate(0, 0, -75);
  scnMgr->getRootSceneNode()->createChildSceneNode()->attachObject(wall2);
  scnMgr->getRootSceneNode()->createChildSceneNode()->attachObject(wall3);
  wall3->getParentSceneNode()->rotate(Ogre::Vector3::UNIT_Z, Ogre::Radian(Ogre::Degree(180)));
  scnMgr->getRootSceneNode()->createChildSceneNode()->attachObject(wall4);
  scnMgr->getRootSceneNode()->createChildSceneNode()->attachObject(wall5);
  wall5->getParentSceneNode()->rotate(Ogre::Vector3::UNIT_X, Ogre::Radian(Ogre::Degree(180)));
  scnMgr->getRootSceneNode()->createChildSceneNode()->attachObject(box1);
  box1->getParentSceneNode()->setPosition(25, -(75 - 22.5), -10);
  box1->getParentSceneNode()->scale(0.45, 0.45, 0.45);
  box1->getParentSceneNode()->rotate(Ogre::Vector3::NEGATIVE_UNIT_Y, Ogre::Radian(Ogre::Degree(20)));
  scnMgr->getRootSceneNode()->createChildSceneNode()->attachObject(light);
  light->getParentSceneNode()->rotate(Ogre::Vector3::UNIT_X, Ogre::Radian(Ogre::Degree(180)));
  light->getParentSceneNode()->translate(0, -1, 0);
  scnMgr->getRootSceneNode()->createChildSceneNode()->attachObject(box2);
  box2->getParentSceneNode()->setPosition(-25, -30, -40);
  box2->getParentSceneNode()->scale(0.4, 0.9, 0.4);
  box2->getParentSceneNode()->rotate(Ogre::Vector3::UNIT_Y, Ogre::Radian(Ogre::Degree(20)));

  // create BVHs for Entities
  BVHroots.resize(8);
  this->addOneEntity(BVHroots[0], wall1, 0);
  this->addOneEntity(BVHroots[1], wall2, 1);
  this->addOneEntity(BVHroots[2], wall3, 2);
  this->addOneEntity(BVHroots[3], wall4, 3);
  this->addOneEntity(BVHroots[4], wall5, 4);
  this->addOneEntity(BVHroots[5], box1, 5);
  this->addOneEntity(BVHroots[6], light, 6);
  this->addOneEntity(BVHroots[7], box2, 7);

  // create BRDF materials
  Materials.resize(8);
  Materials[0] = new DiffuseMaterial(Ogre::Vector3(0.9, 0.9, 0.9));
  Materials[1] = new DiffuseMaterial(Ogre::Vector3(1, 0.0, 0));
  Materials[2] = new DiffuseMaterial(Ogre::Vector3(0, 0, 1));
  Materials[3] = new DiffuseMaterial(Ogre::Vector3(0.9, 0.9, 0.9));
  Materials[4] = new DiffuseMaterial(Ogre::Vector3(0.9, 0.9, 0.9));
  Materials[5] = new DiffuseMaterial(Ogre::Vector3(0.95, 0.95, 0.95));
  Materials[6] = new LightMaterial(Ogre::Vector3(70, 70, 70));
  Materials[7] = new MirrorMaterial();

  // Monte Carlo Path Tracing
  RandomClass randomGen(0.0, 1.0);
  size_t width = 320, height = 320, num_paths = 10000, bounce = 5;
  Ogre::Real pixel_size = 75.0 / width;
  Ogre::Vector3 cam_pos(0, 0, 150);
  Ogre::Vector3 pixelBuf[width][height];
  for (size_t i = 0; i < width; ++i)
    for (size_t j = 0; j < height; ++j)
      pixelBuf[i][j] = Ogre::Vector3::ZERO;
  for (size_t k = 0; k < num_paths; ++k) {
    for (size_t i = 0; i < width; ++i) {
      for (size_t j = 0; j < height; ++j) {
        Ogre::Vector3 origin = Ogre::Vector3(-37.5 + (j + randomGen.rand()) * pixel_size,
                                             37.5 - (i + randomGen.rand()) * pixel_size,
                                             75 + 10);
        Ogre::Vector3 dir = (origin - cam_pos).normalisedCopy();
        Ogre::Ray ray(cam_pos, dir);
        TriangleTestResult result = intersect(ray);
        if (result.hit) {
          pixelBuf[i][j] = (pixelBuf[i][j] * k +
              pathTracing(Ogre::Ray(ray.getPoint(result.t), -dir),
                          result.normal,
                          result.source,
                          bounce)) / (k + 1);
        } else {
          pixelBuf[i][j] = (pixelBuf[i][j] * k) / (k + 1);
        }
      }
    }

    std::cout << k << std::endl;

    std::ofstream outfile("image.ppm", std::ios::out | std::ios::trunc);
    outfile << "P3\n" << width << " " << height << "\n255\n";
    for (size_t i = 0; i < width; ++i)
      for (size_t j = 0; j < height; ++j) {
        int ir = int(255.99 * clamp(pixelBuf[i][j].x));
        int ig = int(255.99 * clamp(pixelBuf[i][j].y));
        int ib = int(255.99 * clamp(pixelBuf[i][j].z));
        outfile << ir << " " << ig << " " << ib << "\n";
      }
    outfile.close();
  }

  for (const auto& BVHroot : BVHroots) delete BVHroot;
  for (const auto& BRDFMaterial : Materials) delete BRDFMaterial;
}

int main(int argc, char* argv[]) {
  BezierApp app;
  app.initApp();
  app.getRoot()->startRendering();
  app.closeApp();
  return 0;
}
