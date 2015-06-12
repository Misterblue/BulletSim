
/*
 * Interface between OpenSimulator/BulletSimStar and an
 * instance of BulletSimStar running as a separate process.
 */

namespace cpp org.opensimulator.bulletsim
namespace csharp OpenSim.Region.Physics.BulletSPlugin

typedef double ffloat

typedef i64 memPtr
typedef memPtr BulletSimStar
typedef memPtr btCollisionObjectStar
typedef memPtr btCollisionShapeStar
typedef memPtr btCompoundShapeStar
typedef memPtr btTypedConstraintStar
typedef memPtr btbroadphaseproxyStar
typedef memPtr voidStar

typedef i32 IDTYPE

struct Vector3 {
    1: required ffloat X;
    2: required ffloat Y;
    3: required ffloat Z
}

struct Quaternion {
    1: required ffloat X;
    2: required ffloat Y;
    3: required ffloat Z;
    4: required ffloat W
}

struct Transform {
    1: required list<double> matrix;    // 3x3 matrix
    2: required Vector3 origin
}

// Information for one object update
struct Update {
    1: required IDTYPE ID;
    2: required Vector3 position;
    3: required Quaterion rotation;
    4: required Vector3 velocity;
    5: required Vector3 acceleration;
    6: required Vector3 angularVelocity
}

// Information for one collision
struct Collision {
    1: required IDTYPE aID;
    2: required IDTYPE bID;
    3: required Vector3 point;
    4: required Vector3 normal;
    5: required ffloat penetration
}

// Information returned by a physics time step
struct StepReturnInfo {
    1: required int updatedEntityCount;
    2: required list<Update> updates;
    3: required int collidersCount;
    4: required list<Collision> collisions
}

struct paramBlock {
    1: required ffloat defaultFriction;
    2: required ffloat defaultDensity;
    3: required ffloat defaultRestitution;
    4: required ffloat collisionMargin;
    5: required ffloat gravity

    6: required ffloat maxPersistantManifoldPoolSize;
    7: required ffloat maxCollisionAlgorithmPoolSize;
    8: required ffloat shouldDisableContactPoolDynamicAllocation;
    9: required ffloat shouldForceUpdateAllAabbs;
    10: required ffloat shouldRandomizeSolverOrder;
    11: required ffloat shouldSplitSimulationIslands;
    12: required ffloat shouldEnableFrictionCaching;
    13: required ffloat numberOfSolverIterations;
    14: required ffloat useSingleSidedMeshes;
    15: required ffloat globalContactBreakingThreshold;

    16: required ffloat physicsLoggingFrames
}

struct HACDParams {
    1: required ffloat maxVerticesPerHull;       // 100
    2: required ffloat minClusters;              // 2
    3: required ffloat compacityWeight;          // 0.1
    4: required ffloat volumeWeight;             // 0.0
    5: required ffloat concavity;                // 100
    6: required ffloat addExtraDistPoints;       // false
    7: required ffloat addNeighboursDistPoints;  // false
    8: required ffloat addFacesPoints;           // false
    9: required ffloat shouldAdjustCollisionMargin;  // false
    // VHACD
    10: required ffloat whichHACD;                // zero if Bullet HACD, non-zero says VHACD
    // http://kmamou.blogspot.ca/2014/12/v-hacd-20-parameters-description.html
    11: required ffloat vHACDresolution;          // max number of voxels generated during voxelization stage
    12: required ffloat vHACDdepth;               // max number of clipping stages
    13: required ffloat vHACDconcavity;           // maximum concavity
    14: required ffloat vHACDplaneDownsampling;   // granularity of search for best clipping plane
    15: required ffloat vHACDconvexHullDownsampling;  // precision of hull gen process
    16: required ffloat vHACDalpha;               // bias toward clipping along symmetry planes
    17: required ffloat vHACDbeta;                // bias toward clipping along revolution axis
    18: required ffloat vHACDdelta;               // bias toward clipping within local convex shape
    19: required ffloat vHACDgamma;               // max concavity when merging
    20: required ffloat vHACDpca;                 // on/off normalizing mesh before decomp
    21: required ffloat vHACDmode;                // 0:voxel based, 1: tetrahedron based
    22: required ffloat vHACDmaxNumVerticesPerCH; // max triangles per convex hull
    23: required ffloat vHACDminVolumePerCH;      // sampling of generated convex hulls
    24: required ffloat vHACDconvexHullApprox;    // approximate hulls to accelerate computation
    25: required ffloat vHACDoclAcceleration     // use OpenCL
}

struct SweepHit {
    1: required IDTYPE id;
    2: required ffloat fraction;
    3: required Vector3 normal;
    4: required Vector3 point
}

struct RaycastHit {
    1: required IDTYPE id;
    2: required ffloat fraction;
    3: required Vector3 Normal
}

service BulletSimStarThrift {
    void Initialize2(1: required Vector3 maxPosition,
                    2: required paramBlock parms,
                    3: required int maxCollisions,
                    4: required int maxUpdates
    ),

    void UpdateParameter2(
                    1: required memPtr sim,
                    2: required IDTYPE localID,
                    3: required string parm,
                    4: required double value
    ),

    void Shutdown2(
                    1: required BulletSimStar sim
    ),

    void ResetBroadphasePool(
    				1: required BulletSimStar sim
    ),
    void ResetConstraintSolver(
    				1: required BulletSimStar sim
    ),

    StepReturnInfo PhysicsStep2(
    				1: required BulletSimStar sim,
    				2: required ffloat timeStep,
    				3: required int maxSubSteps,
    				4: required ffloat fixedTimeStep,
    ),
    bool PushUpdate2(
    				1: required btCollisionShapeStar obj
    ),
    btCollisionShapeStar CreateMeshShape2(
    				1: required BulletSimStar sim,
    				2: required int indicesCount,
    				3: required list<int> indices,
    				4: required int verticesCount,
    				5: required list<double> vertices 
    ),
    btCollisionShapeStar CreateGImpactShape2(
    				1: required BulletSimStar sim,
    				2: required int indicesCount,
    				3: required list<int> indices,
    				4: required int verticesCount,
    				5: required list<double> vertices 
    ),
    btCollisionShapeStar CreateHullShape2(
    				1: required BulletSimStar sim,
    				2: required int hullCount,
    				3: required list<double> hulls 
    ),
    btCollisionShapeStar BuildHullShapeFromMesh2(
    				1: required BulletSimStar sim,
    				2: required btCollisionShapeStar mesh,
    				3: required HACDParams parms
    ),
    btCollisionShapeStar BuildConvexHullShapeFromMesh2(
    				1: required BulletSimStar sim,
    				2: required btCollisionShapeStar mesh
    ),
    btCollisionShapeStar CreateConvexHullShape2(
    				1: required BulletSimStar sim,
    				2: required int indicesCount,
    				3: required list<int> indices,
    				4: required int verticesCount,
    				5: required list<double> vertices 
    ),
    btCollisionShapeStar CreateCompoundShape2(
    				1: required BulletSimStar sim,
    				2: required bool enableDynamicAabbTree
    ),
    int GetNumberOfCompoundChildren2(
    				1: required btCompoundShapeStar cShape
    ),
    void AddChildShapeToCompoundShape2(
    				1: required btCompoundShapeStar cShape,
    				2: required btCollisionShapeStar addShape,
    				3: required Vector3 relativePosition,
    				4: required Quaternion relativeRotation
    ),
    btCollisionShapeStar GetChildShapeFromCompoundShapeIndex2(
    				1: required btCompoundShapeStar cShape,
    				2: required int ii
    ),
    void RemoveChildShapeFromCompoundShape2(
    				1: required btCompoundShapeStar cShape,
    				2: required btCollisionShapeStar removeShape
    ),
    btCollisionShapeStar RemoveChildShapeFromCompoundShapeIndex2(
    				1: required btCompoundShapeStar cShape,
    				2: required int ii
    ),
    void RecalculateCompoundShapeLocalAabb2(
    				1: required btCompoundShapeStar cShape
    ),
    void UpdateChildTransform2(
    				1: required btCompoundShapeStar cShape,
    				2: required int childIndex,
    				3: required Vector3 pos,
    				4: required Quaternion rot,
    				5: required bool shouldRecalculateLocalAabb
    ),
    Vector3 GetCompoundChildPosition2(
    				1: required btCompoundShapeStar cShape,
    				2: required int childIndex
    ),
    Quaternion GetCompoundChildOrientation2(
    				1: required btCompoundShapeStar cShape,
    				2: required int childIndex
    ),
    btCollisionShapeStar BuildNativeShape2(
    				1: required BulletSimStar sim,
    				2: required ShapeData shapeData
    ),
    bool IsNativeShape2(
    				1: required btCollisionShapeStar shape
    ),
    void SetShapeCollisionMargin(
    				1: required btCollisionShapeStar shape,
    				2: required ffloat margin
    ),
    btCollisionShapeStar BuildCapsuleShape2(
    				1: required BulletSimStar sim,
    				2: required ffloat radius,
    				3: required ffloat height,
    				4: required Vector3 scale
    ),
    bool DeleteCollisionShape2(
    				1: required BulletSimStar sim,
    				2: required btCollisionShapeStar shape
    ),
    btCollisionShapeStar DuplicateCollisionShape2(
    				1: required BulletSimStar sim,
    				2: required btCollisionShapeStar src,
    				3: required IDTYPE id
    ),
    int GetBodyType2(
    				1: required btCollisionShapeStar obj
    ),
    btCollisionShapeStar CreateBodyFromShape2(
    				1: required BulletSimStar sim,
    				2: required btCollisionShapeStar shape,
    				3: required IDTYPE id,
    				4: required Vector3 pos,
    				5: required Quaternion rot
    ),
    btCollisionShapeStar CreateBodyWithDefaultMotionState2(
    				1: required btCollisionShapeStar shape,
    				2: required IDTYPE id,
    				3: required Vector3 pos,
    				4: required Quaternion rot
    ),
    btCollisionShapeStar CreateGhostFromShape2(
    				1: required BulletSimStar sim,
    				2: required btCollisionShapeStar shape,
    				3: required IDTYPE id,
    				4: required Vector3 pos,
    				5: required Quaternion rot
    ),
    void DestroyObject2(
    				1: required BulletSimStar sim,
    				2: required btCollisionShapeStar obj
    ),
    btCollisionShapeStar CreateTerrainShape2(
    				1: required IDTYPE id,
    				2: required Vector3 size,
    				3: required ffloat minHeight,
    				4: required ffloat maxHeight,
    				5: required list<double> heightmap,
    				6: required ffloat scalefactor,
    				7: required ffloat collisionmargin
    ),
    btcollisionshapestar creategroundplaneshape2(
    				1: required idtype id,
    				2: required ffloat height,
    				3: required ffloat collisionmargin
    ),
    bttypedconstraintstar create6dofconstraint2(
    				1: required bulletsimstar sim,
    				2: required btcollisionshapestar obj1,
    				3: required btcollisionshapestar obj2,
    				4: required vector3 frame1loc,
    				5: required quaternion frame1rot,
    				6: required vector3 frame2loc,
    				7: required quaternion frame2rot,
    				8: required bool uselinearreferenceframea,
    				9: required bool disablecollisionsbetweenlinkedbodies
    ),
    bttypedconstraintstar create6dofconstrainttopoint2(
    				1: required bulletsimstar sim,
    				2: required btcollisionshapestar obj1,
    				3: required btcollisionshapestar obj2,
    				4: required vector3 joinpoint,
    				5: required bool uselinearreferenceframea,
    				6: required bool disablecollisionsbetweenlinkedbodies
    ),
    bttypedconstraintstar create6dofconstraintfixed2(
    				1: required bulletsimstar sim,
    				2: required btcollisionshapestar obj1,
    				3: required vector3 frameinbloc,
    				4: required quaternion frameinbrot,
    				5: required bool uselinearreferenceframeb,
    				6: required bool disablecollisionsbetweenlinkedbodies
    ),
    bttypedconstraintstar create6dofspringconstraint2(
    				1: required bulletsimstar sim,
    				2: required btcollisionshapestar obj1,
    				3: required btcollisionshapestar obj2,
    				4: required vector3 frame1loc,
    				5: required quaternion frame1rot,
    				6: required vector3 frame2loc,
    				7: required quaternion frame2rot,
    				8: required bool uselinearreferenceframea,
    				9: required bool disablecollisionsbetweenlinkedbodies
    ),
    bttypedconstraintstar createhingeconstraint2(
    				1: required bulletsimstar sim,
    				2: required btcollisionshapestar obj1,
    				3: required btcollisionshapestar obj2,
    				4: required vector3 pivotina,
    				5: required vector3 pivotinb,
    				6: required vector3 axisina,
    				7: required vector3 axisinb,
    				8: required bool usereferenceframea,
    				9: required bool disablecollisionsbetweenlinkedbodies
    ),
    bttypedconstraintstar createsliderconstraint2(
    				1: required bulletsimstar sim,
    				2: required btcollisionshapestar obj1,
    				3: required btcollisionshapestar obj2,
    				4: required vector3 frame1loc,
    				5: required quaternion frame1rot,
    				6: required vector3 frame2loc,
    				7: required quaternion frame2rot,
    				8: required bool uselinearreferenceframea,
    				9: required bool disablecollisionsbetweenlinkedbodies
    ),
    bttypedconstraintstar createconetwistconstraint2(
    				1: required bulletsimstar sim,
    				2: required btcollisionshapestar obj1,
    				3: required btcollisionshapestar obj2,
    				4: required vector3 frame1loc,
    				5: required quaternion frame1rot,
    				6: required vector3 frame2loc,
    				7: required quaternion frame2rot,
    				8: required bool disablecollisionsbetweenlinkedbodies
    ),
    bttypedconstraintstar creategearconstraint2(
    				1: required bulletsimstar sim,
    				2: required btcollisionshapestar obj1,
    				3: required btcollisionshapestar obj2,
    				4: required vector3 axisina,
    				5: required vector3 axisinb,
    				6: required vector3 frame2loc,
    				7: required quaternion frame2rot,
    				8: required ffloat ratio,
    				9: required bool disablecollisionsbetweenlinkedbodies
    ),
    bttypedconstraintstar createpoint2pointconstraint2(
    				1: required bulletsimstar sim,
    				2: required btcollisionshapestar obj1,
    				3: required btcollisionshapestar obj2,
    				4: required vector3 pivotina,
    				5: required vector3 pivotinb,
    				6: required bool disablecollisionsbetweenlinkedbodies
    ),
    setframes2(
    				1: required bttypedconstraintstar constrain,
    				2: required vector3 framea,
    				3: required quaternion framearot,
    				4: required vector3 frameb,
    				5: required quaternion framebrot
    ),
    void setconstraintenable2(
    				1: required bttypedconstraintstar constrain,
    				2: required ffloat truefalse
    ),
    void setconstraintnumsolveriterations2(
    				1: required bttypedconstraintstar constrain,
    				2: required ffloat iterations
    ),
    bool setlinearlimits2(
    				1: required bttypedconstraintstar constrain,
    				2: required vector3 low,
    				3: required vector3 high
    ),
    bool setangularlimits2(
    				1: required bttypedconstraintstar constrain,
    				2: required vector3 low,
    				3: required vector3 high
    ),
    bool useframeoffset2(
    				1: required bttypedconstraintstar constrain,
    				2: required ffloat enable
    ),
    bool translationallimitmotor2(
    				1: required bttypedconstraintstar constrain,
    				2: required ffloat enable,
    				3: required ffloat targetvelocity,
    				4: required ffloat maxmotorforce
    ),
    bool setbreakingimpulsethreshold2(
    				1: required bttypedconstraintstar constrain,
    				2: required ffloat thresh
    ),
    bool constraintsetaxis2(
    				1: required bttypedconstraintstar constrain,
    				2: required vector3 axisa,
    				3: required vector3 axisb
    ),
    bool constrainthingesetlimit2(
    				1: required bttypedconstraintstar constrain,
    				2: required ffloat low,
    				3: required ffloat high,
    				4: required ffloat softness,
    				5: required ffloat bias,
    				6: required ffloat relaxation
    ),
    bool constraintspringenable2(
    				1: required bttypedconstraintstar constrain,
    				2: required int index,
    				3: required bool onoff
    ),
    bool constraintspringsetequilibriumpoint2(
    				1: required bttypedconstraintstar constrain,
    				2: required int index,
    				3: required ffloat eqpoint
    ),
    bool constraintspringsetstiffness2(
    				1: required bttypedconstraintstar constrain,
    				2: required int index,
    				3: required ffloat stiffness
    ),
    bool constraintspringsetdamping2(
    				1: required bttypedconstraintstar constrain,
    				2: required int index,
    				3: required ffloat damping
    ),
    bool constraintslidersetlimits2(
    				1: required bttypedconstraintstar constrain,
    				2: required int upperlower,
    				3: required int linang,
    				4: required ffloat val
    ),
    bool constraintsliderset2(
    				1: required bttypedconstraintstar constrain,
    				2: required int softrestdamp,
    				3: required int dirlimortho,
    				4: required int linang,
    				5: required ffloat val
    ),
    bool constraintslidermotorenable2(
    				1: required bttypedconstraintstar constrain,
    				2: required int linang,
    				3: required ffloat numerictruefalse
    ),
    bool constraintslidermotor2(
    				1: required bttypedconstraintstar constrain,
    				2: required int forcevel,
    				3: required int linang,
    				4: required ffloat val
    ),
    bool calculatetransforms2(
    				1: required bttypedconstraintstar constrain
    ),
    bool setconstraintparam2(
    				1: required bttypedconstraintstar constrain,
    				2: required int paramindex,
    				3: required ffloat value,
    				4: required int axis
    ),
    bool destroyconstraint2(
    				1: required bulletsimstar sim,
    				2: required bttypedconstraintstar constrain
    ),
    // =======================================
    void updatesingleaabb2(
    				1: required bulletsimstar world,
    				2: required btcollisionshapestar obj
    ),
    void updateaabbs2(
    				1: required bulletsimstar world
    ),
    bool getforceupdateallaabbs2(
    				1: required bulletsimstar world
    ),
    void setforceupdateallaabbs2(
    				1: required bulletsimstar world,
    				2: required bool forceupdateallaabbs
    ),
    // ========================================
    bool addobjecttoworld2(
    				1: required bulletsimstar sim,
    				2: required btcollisionshapestar obj
    ),
    bool removeobjectfromworld2(
    				1: required bulletsimstar sim,
    				2: required btcollisionshapestar obj
    ),
    bool clearcollisionproxycache2(
    				1: required bulletsimstar sim,
    				2: required btcollisionshapestar obj
    ),
    bool addconstrainttoworld2(
    				1: required bulletsimstar sim,
    				2: required bttypedconstraintstar constrain,
    				3: required bool disablecollisionsbetweenlinkedbodies
    ),
    bool removeconstraintfromworld2(
    				1: required bulletsimstar sim,
    				2: required bttypedconstraintstar constrain
    ),
    // ========================================
    vector3 getanisotropicfriction2(
    				1: required btcollisionshapestar obj
    ),
    void setanisotropicfriction2(
    				1: required btcollisionshapestar obj,
    				2: required vector3 africt
    ),
    bool hasanisotropicfriction2(
    				1: required btcollisionshapestar obj
    ),
    void setcontactprocessingthreshold2(
    				1: required btcollisionshapestar obj,
    				2: required ffloat threshold
    ),
    ffloat getcontactprocessingthreshold2(
    				1: required btcollisionshapestar obj
    ),
    bool isstaticobject2(
    				1: required btcollisionshapestar obj
    ),
    bool iskinematicobject2(
    				1: required btcollisionshapestar obj
    ),
    bool isstaticorkinematicobject2(
    				1: required btcollisionshapestar obj
    ),
    bool hascontactresponse2(
    				1: required btcollisionshapestar obj
    ),
    void setcollisionshape2(
    				1: required bulletsimstar sim,
    				2: required btcollisionshapestar obj,
    				3: required btcollisionshapestar shape
    ),
    btcollisionshapestar getcollisionshape2(
    				1: required btcollisionshapestar obj
    ),
    int getactivationstate2(
    				1: required btcollisionshapestar obj
    ),
    void setactivationstate2(
    				1: required btcollisionshapestar obj,
    				2: required int state
    ),
    void setdeactivationtime2(
    				1: required btcollisionshapestar obj,
    				2: required ffloat dtime
    ),
    ffloat getdeactivationtime2(
    				1: required btcollisionshapestar obj
    ),
    void forceactivationstate2(
    				1: required btcollisionshapestar obj,
    				2: required int newstate
    ),
    void activate2(
    				1: required btcollisionshapestar obj,
    				2: required bool forceactivation
    ),
    bool isactive2(
    				1: required btcollisionshapestar obj
    ),
    void setrestitution2(
    				1: required btcollisionshapestar obj,
    				2: required ffloat val
    ),
    ffloat getrestitution2(
    				1: required btcollisionshapestar obj
    ),
    void setfriction2(
    				1: required btcollisionshapestar obj,
    				2: required ffloat val
    ),
    ffloat getfriction2(
    				1: required btcollisionshapestar obj
    ),
    void setworldtransform2(
    				1: required btcollisionshapestar obj,
    				2: required transform& trans
    ),
    transform getworldtransform2(
    				1: required btcollisionshapestar obj
    ),
    vector3 getposition2(
    				1: required btcollisionshapestar obj
    ),
    quaternion getorientation2(
    				1: required btcollisionshapestar obj
    ),
    void settranslation2(
    				1: required btcollisionshapestar obj,
    				2: required vector3 position,
    				3: required quaternion rotation
    ),
    btbroadphaseproxyStar getbroadphasehandle2(
    				1: required btcollisionshapestar obj
    ),
    void setbroadphasehandle2(
    				1: required btcollisionshapestar obj,
    				2: required btbroadphaseproxyStar proxy
    ),
    transform getinterpolationworldtransform2(
    				1: required btcollisionshapestar obj
    ),
    void setinterpolationworldtransform2(
    				1: required btcollisionshapestar obj,
    				2: required transform trans
    ),
    void setinterpolationlinearvelocity2(
    				1: required btcollisionshapestar obj,
    				2: required vector3 vel
    ),
    void setinterpolationangularvelocity2(
    				1: required btcollisionshapestar obj,
    				2: required vector3 ang
    ),
    void setinterpolationvelocity2(
    				1: required btcollisionshapestar obj,
    				2: required vector3 lin,
    				3: required vector3 ang
    ),
    vector3 getinterpolationlinearvelocity2(
    				1: required btcollisionshapestar obj
    ),
    vector3 getinterpolationangularvelocity2(
    				1: required btcollisionshapestar obj
    ),
    ffloat gethitfraction2(
    				1: required btcollisionshapestar obj
    ),
    void sethitfraction2(
    				1: required btcollisionshapestar obj,
    				2: required ffloat val
    ),
    int getcollisionflags2(
    				1: required btcollisionshapestar obj
    ),
    uint32_t setcollisionflags2(
    				1: required btcollisionshapestar obj,
    				2: required uint32_t flags
    ),
    uint32_t addtocollisionflags2(
    				1: required btcollisionshapestar obj,
    				2: required uint32_t flags
    ),
    uint32_t removefromcollisionflags2(
    				1: required btcollisionshapestar obj,
    				2: required uint32_t flags
    ),
    ffloat getccdsweptsphereradius2(
    				1: required btcollisionshapestar obj
    ),
    void setccdsweptsphereradius2(
    				1: required btcollisionshapestar obj,
    				2: required ffloat val
    ),
    ffloat getccdmotionthreshold2(
    				1: required btcollisionshapestar obj
    ),
    ffloat getsquareccdmotionthreshold2(
    				1: required btcollisionshapestar obj
    ),
    void setccdmotionthreshold2(
    				1: required btcollisionshapestar obj,
    				2: required ffloat val
    ),
    voidstar getuserpointer2(
    				1: required btcollisionshapestar obj
    ),
    void setuserpointer2(
    				1: required btcollisionshapestar obj,
                    2: voidstar ptr
    ),
    // ==================================================================================
    // btrigidbody methods
    // these are in the order found in btrigidbody.h
    void applygravity2(
    				1: required btcollisionshapestar obj
    ),
    void setgravity2(
    				1: required btcollisionshapestar obj,
    				2: required vector3 grav
    ),
    				1: required vector3 getgravity2(
    				2: required btcollisionshapestar obj
    ),
    void setdamping2(
    				1: required btcollisionshapestar obj,
    				2: required ffloat lin_damping,
    				3: required ffloat ang_damping
    ),
    void setlineardamping2(
    				1: required btcollisionshapestar obj,
    				2: required ffloat lin_damping
    ),
    void setangulardamping2(
    				1: required btcollisionshapestar obj,
    				2: required ffloat ang_damping
    ),
    ffloat getlineardamping2(
    				1: required btcollisionshapestar obj
    ),
    ffloat getangulardamping2(
    				1: required btcollisionshapestar obj
    ),
    ffloat getlinearsleepingthreshold2(
    				1: required btcollisionshapestar obj
    ),
    ffloat getangularsleepingthreshold2(
    				1: required btcollisionshapestar obj
    ),
    void applydamping2(
    				1: required btcollisionshapestar obj,
    				2: required ffloat timestep
    ),
    void setmassprops2(
    				1: required btcollisionshapestar obj,
    				2: required ffloat mass,
    				3: required vector3 inertia
    ),
    vector3 getlinearfactor2(
    				1: required btcollisionshapestar obj
    ),
    void setlinearfactor2(
    				1: required btcollisionshapestar obj,
    				2: required vector3 fact
    ),
    void setcenterofmasstransform2(
    				1: required btcollisionshapestar obj,
    				2: required transform trans
    ),
    void setcenterofmassbyposrot2(
    				1: required btcollisionshapestar obj,
    				2: required vector3 pos,
    				3: required quaternion rot
    ),
    void applycentralforce2(
    				1: required btcollisionshapestar obj,
    				2: required vector3 force
    ),
    void setobjectforce2(
    				1: required btcollisionshapestar obj,
    				2: required vector3 force
    ),
    vector3 gettotalforce2(
    				1: required btcollisionshapestar obj
    ),
    vector3 gettotaltorque2(
    				1: required btcollisionshapestar obj
    ),
    vector3 getinvinertiadiaglocal2(
    				1: required btcollisionshapestar obj
    ),
    void setinvinertiadiaglocal2(
    				1: required btcollisionshapestar obj,
    				2: required vector3 inert
    ),
    void setsleepingthresholds2(
    				1: required btcollisionshapestar obj,
    				2: required ffloat lin_threshold,
    				3: required ffloat ang_threshold
    ),
    void applytorque2(
    				1: required btcollisionshapestar obj,
    				2: required vector3 force
    ),
    void applyforce2(
    				1: required btcollisionshapestar obj,
    				2: required vector3 force,
    				3: required vector3 pos
    ),
    void applycentralimpulse2(
    				1: required btcollisionshapestar obj,
    				2: required vector3 force
    ),
    void applytorqueimpulse2(
    				1: required btcollisionshapestar obj,
    				2: required vector3 force
    ),
    void applyimpulse2(
    				1: required btcollisionshapestar obj,
    				2: required vector3 force,
    				3: required vector3 pos
    ),
    void clearforces2(
    				1: required btcollisionshapestar obj
    ),
    void clearallforces2(
    				1: required btcollisionshapestar obj
    ),
    void updateinertiatensor2(
    				1: required btcollisionshapestar obj
    ),
    vector3 getcenterofmassposition2(
    				1: required btcollisionshapestar obj
    ),
    quaternion getorientation2(
    				1: required btcollisionshapestar obj
    ),
    transform getcenterofmasstransform2(
    				1: required btcollisionshapestar obj
    ),
    vector3 getlinearvelocity2(
    				1: required btcollisionshapestar obj
    ),
    vector3 getangularvelocity2(
    				1: required btcollisionshapestar obj
    ),
    void setlinearvelocity2(
    				1: required btcollisionshapestar obj,
    				2: required vector3 velocity
    ),
    void setangularvelocity2(
    				1: required btcollisionshapestar obj,
    				2: required vector3 angularvelocity
    ),
    vector3 getvelocityinlocalpoint2(
    				1: required btcollisionshapestar obj,
    				2: required vector3 pos
    ),
    void translate2(
    				1: required btcollisionshapestar obj,
    				2: required vector3 trans
    ),
    void updatedeactivation2(
    				1: required btcollisionshapestar obj,
    				2: required ffloat timestep
    ),
    bool wantssleeping2(
    				1: required btcollisionshapestar obj
    ),
    void setangularfactor2(
    				1: required btcollisionshapestar obj,
    				2: required ffloat fact
    ),
    void setangularfactorv2(
    				1: required btcollisionshapestar obj,
    				2: required vector3 fact
    ),
    vector3 getangularfactor2(
    				1: required btcollisionshapestar obj
    ),
    bool isinworld2(
    				1: required btcollisionshapestar obj
    ),
    void addconstraintref2(
    				1: required btcollisionshapestar obj,
    				2: required bttypedconstraintstar constrain
    ),
    void removeconstraintref2(
    				1: required btcollisionshapestar obj,
    				2: required bttypedconstraintstar constrain
    ),
    bttypedconstraintstar getconstraintref2(
    				1: required btcollisionshapestar obj,
    				2: required int index
    ),
    int getnumconstraintrefs2(
    				1: required btcollisionshapestar obj
    ),
    // ========================================
    // btcollisionshapestar methods and related
    ffloat getangularmotiondisc2(
    				1: required btcollisionshapestar shape
    ),
    ffloat getcontactbreakingthreshold2(
    				1: required btcollisionshapestar shape,
    				2: required ffloat defaultfactor
    ),
    bool isployhedral2(
    				1: required btcollisionshapestar shape
    ),
    bool isconvex2d2(
    				1: required btcollisionshapestar shape
    ),
    bool isconvex2(
    				1: required btcollisionshapestar shape
    ),
    bool isnonmoving2(
    				1: required btcollisionshapestar shape
    ),
    bool isconcave2(
    				1: required btcollisionshapestar shape
    ),
    bool iscompound2(
    				1: required btcollisionshapestar shape
    ),
    bool issoftbody2(
    				1: required btcollisionshapestar shape
    ),
    bool isinfinite2(
    				1: required btcollisionshapestar shape
    ),
    void setlocalscaling2(
    				1: required btcollisionshapestar shape,
    				2: required vector3 scale
    ),
    vector3 getlocalscaling2(
    				1: required btcollisionshapestar shape
    ),
    vector3 calculatelocalinertia2(
    				1: required btcollisionshapestar shape,
    				2: required ffloat mass
    ),
    int getshapetype2(
    				1: required btcollisionshapestar shape
    ),
    void setmargin2(
    				1: required btcollisionshapestar shape,
    				2: required ffloat val
    ),
    ffloat getmargin2(
    				1: required btcollisionshapestar shape
    ),
    bool setcollisiongroupmask2(
    				1: required btcollisionshapestar obj,
    				2: required i32 group,
    				3: required i32 mask
    ),

    // =====================================================================
    SweepHit ConvexSweepTest2(
    				1: required BulletSimStar world,
    				2: required i32 id,
    				3: required Vector3 from,
    				4: required Vector3 to,
    				5: required ffloat extraMargin
    ),
    RaycastHit RayTest2(
    				1: required BulletSimStar world,
    				2: required i32 id,
    				3: required Vector3 from,
    				4: required Vector3 to
    ),
    Vector3 RecoverFromPenetration2(
    				1: required BulletSimStar world,
    				2: required i32 id
    ),

    // Debugging
    // Dump a btCollisionObject and even more if it's a btRigidBody.
    void DumpRigidBody2(
    				1: required BulletSimStar sim,
    				2: required btCollisionShapeStar obj
    ),
    void DumpCollisionShape2(
    				1: required BulletSimStar sim,
    				2: required btCollisionShapeStar shape
    ),
    void DumpFrameInfo(
    				1: required BulletSimStar sim,
    				2: required memPtr type,
    				3: required btTransform& frameInA,
    				4: required btTransform& frameInB
    ),
    void Dump6DofInfo(
    				1: required BulletSimStar sim,
    				2: required memPtr type,
    				3: required btGeneric6DofConstraintStar constrain
    ),
    void DumpConstraint2(
    				1: required BulletSimStar sim,
    				2: required btTypedConstraintStar constrain
    ),
    void DumpAllInfo2(
    				1: required BulletSimStar sim
    ),
    void DumpActivationInfo2(
    				1: required BulletSimStar sim
    ),
