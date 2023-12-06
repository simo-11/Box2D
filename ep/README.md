# Introduction
ep is main contribution of this fork of Box2D.
It contains various attempts to handle elasticity and plasticity within Box2D.

# Changes to core files
These are written for myself to help getting synced after long breaks, last break was 5 years.
Put at least comment like // ep
 * b2_body.h
```
include/box2d $ git diff -b upstream/main main -- *.h
diff --git a/include/box2d/b2_body.h b/include/box2d/b2_body.h
index 16b2bb0..b62391b 100644
--- a/include/box2d/b2_body.h
+++ b/include/box2d/b2_body.h
@@ -398,6 +398,9 @@ private:
        friend class b2FrictionJoint;
        friend class b2GearJoint;
        friend class b2MotorJoint;
+       friend class b2ElasticPlasticJoint; // ep
+       friend class b2RigidPlasticJoint; // ep
+       friend class b2ImpulseInitializer; // ep
        friend class b2MouseJoint;
```
 * b2_common.h
```
+// ep
+#define EP_LOG 1
+/// code modifies epLogActive
+/// epLogEnabled is controlled by UI
+extern bool epLogActive, epLogEnabled;
+/// Logging function.
+void epLog(const char* string, ...);
+void epLogClose();
```
 * b2_draw.h
```
+       // ep
+       void SetInvDt(float val) { m_inv_dt = val; }
+       float GetInvDt() { return m_inv_dt; }
+       // forceScale should be set so that maximum force is visually helpful
+       void SetForceScale(float scale) { m_forceScale = scale; }
+       float GetForceScale() { return m_forceScale; }
+       void SetMomentScale(float scale) { m_momentScale = scale; }
+       float GetMomentScale() { return m_momentScale; }
 protected:
        uint32 m_drawFlags;
+       // ep
+       float m_inv_dt;
+       float m_forceScale, m_momentScale;
 * 
``` b2_joint.h
diff --git a/include/box2d/b2_joint.h b/include/box2d/b2_joint.h
index 586b13a..9aa8f28 100644
--- a/include/box2d/b2_joint.h
+++ b/include/box2d/b2_joint.h
@@ -44,7 +44,9 @@ enum b2JointType
        e_wheelJoint,
     e_weldJoint,
        e_frictionJoint,
-       e_motorJoint
+       e_motorJoint,
+       e_elasticPlasticJoint, // ep
+       e_rigidPlasticJoint // ep
```
 * b2_time_step.h
```
        float solveInit;
+       float initImpulse; // ep
        float solveVelocity;
```
 * b2_world.h
```
        friend class b2Controller;
+       friend class b2ElasticPlasticJoint;
```
 * added b2ep_joint.h
 * src/dynamics
```
Box2D/src/dynamics $ git diff -b upstream/main main -- * | less
diff --git a/src/dynamics/b2_island.cpp b/src/dynamics/b2_island.cpp
index 3441305..8ab3900 100644
--- a/src/dynamics/b2_island.cpp
+++ b/src/dynamics/b2_island.cpp
@@ -185,6 +185,18 @@ b2Island::~b2Island()
        m_allocator->Free(m_bodies);
 }

+// ep
+namespace {
+       bool doInitImpulses = false;
+}
+bool b2Island::IsInitImpulses() {
+       return doInitImpulses;
+}
+void b2Island::SetInitImpulses(bool value) {
+       doInitImpulses = value;
+}
+// end ep
+
diff --git a/src/dynamics/b2_island.h b/src/dynamics/b2_island.h
index 2e28a35..605a6ba 100644
       void Solve(b2Profile* profile, const b2TimeStep& step, const b2Vec2& gravity, bool allowSleep);
+       void InitRigidPlasticJoints(b2SolverData& solverData, const b2Vec2& gravity); // ep
+       static void SetInitImpulses(bool);
+       static bool IsInitImpulses();
+       int32 nonDynamicBodyCount, startJointCount, epCount;
+       b2Body** ndbStack; // non dynamic bodies
+       b2ElasticPlasticJoint** sjStack; // corresponding starting joints
+       b2ElasticPlasticJoint** epStack; // all rigid plastic ep joints
+       void InitEpStacks();
+       void UpdateRigidPlasticJoints(b2SolverData& solverData);
+       // end ep

diff --git a/src/dynamics/b2_joint.cpp b/src/dynamics/b2_joint.cpp
index 41addbb..41eff44 100644

@@ -162,6 +163,22 @@ b2Joint* b2Joint::Create(const b2JointDef* def, b2BlockAllocator* allocator)
                        joint = new (mem) b2MotorJoint(static_cast<const b2MotorJointDef*>(def));
                }
                break;
+               // ep
+       case e_elasticPlasticJoint:
+       {
+               void* mem = allocator->Allocate(sizeof(b2ElasticPlasticJoint));
+               joint = new (mem)b2ElasticPlasticJoint
+               (static_cast<const b2ElasticPlasticJointDef*>(def));
+       }
+       break;
+
+       case e_rigidPlasticJoint:
+       {
+               void* mem = allocator->Allocate(sizeof(b2RigidPlasticJoint));
+               joint = new (mem)b2RigidPlasticJoint
+               (static_cast<const b2RigidPlasticJointDef*>(def));
+       }
+       break;

        default:
                b2Assert(false);
@@ -215,6 +232,13 @@ void b2Joint::Destroy(b2Joint* joint, b2BlockAllocator* allocator)
        case e_motorJoint:
                allocator->Free(joint, sizeof(b2MotorJoint));
                break;
+               //ep
+       case e_elasticPlasticJoint:
+               allocator->Free(joint, sizeof(b2ElasticPlasticJoint));
+               break;
+       case e_rigidPlasticJoint:
+               allocator->Free(joint, sizeof(b2RigidPlasticJoint));
+               break;
```
 * b2ep_joint.cpp added
 * in common
```
Box2D/src/common $ git diff -b upstream/main main -- * | less
 b2Draw::b2Draw()
 {
        m_drawFlags = 0;
+       // ep
+       m_inv_dt=0;
+       m_forceScale = 0;
+       m_momentScale=0;
+
diff --git a/src/common/b2_settings.cpp b/src/common/b2_settings.cpp
index dde28bb..d7a9cbc 100644
--- a/src/common/b2_settings.cpp
+++ b/src/common/b2_settings.cpp
@@ -72,3 +72,51 @@ void b2CloseDump()
        fclose(b2_dumpFile);
        b2_dumpFile = nullptr;
 }
+
+// ep
+FILE* fout = NULL;
+int epLogBytes;
+int maxEpLogBytes = 1 << 23;
+void epLogClose() {
...
+void epLog(const char* string, ...) {
...
+bool epLogActive = true;
+bool epLogEnabled = false;
```
 * CMake
```
 simon@MSI MINGW64 ~/github/Box2D/src (main)
$ git diff -b upstream/main main -- CMakeLists.txt
diff --git a/src/CMakeLists.txt b/src/CMakeLists.txt
index 7b76c78..5ea012e 100644
--- a/src/CMakeLists.txt
+++ b/src/CMakeLists.txt
@@ -39,6 +39,7 @@ set(BOX2D_SOURCE_FILES
        dynamics/b2_joint.cpp
+       dynamics/b2ep_joint.cpp
        dynamics/b2_motor_joint.cpp
```


More details in https://docs.google.com/document/d/1oACqdqwr3fm4IfKLPMXScVii1fb_uPqTpFyLie3QOy4/
