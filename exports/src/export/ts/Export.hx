package export.ts;

import oimo.dynamics.common.DebugDraw;
import oimo.collision.broadphase.BroadPhase;
import oimo.collision.geometry.Geometry;
import oimo.collision.geometry.ConvexGeometry;
import oimo.collision.broadphase.BroadPhaseProxyCallback;
import oimo.collision.broadphase.BroadPhaseType;
import oimo.collision.broadphase.Proxy;
import oimo.collision.broadphase.ProxyPair;
import oimo.collision.broadphase.bruteforce.BruteForceBroadPhase;
import oimo.collision.broadphase.bvh.BvhBroadPhase;
import oimo.collision.broadphase.bvh.BvhInsertionStrategy;
import oimo.collision.broadphase.bvh.BvhNode;
import oimo.collision.broadphase.bvh.BvhProxy;
import oimo.collision.broadphase.bvh.BvhStrategy;
import oimo.collision.broadphase.bvh.BvhTree;
import oimo.collision.geometry.Aabb;
import oimo.collision.geometry.BoxGeometry;
import oimo.collision.geometry.CapsuleGeometry;
import oimo.collision.geometry.ConeGeometry;
import oimo.collision.geometry.ConvexHullGeometry;
import oimo.collision.geometry.CylinderGeometry;
import oimo.collision.geometry.GeometryType;
import oimo.collision.geometry.RayCastHit;
import oimo.collision.geometry.SphereGeometry;
import oimo.collision.narrowphase.CollisionMatrix;
import oimo.collision.narrowphase.DetectorResult;
import oimo.collision.narrowphase.DetectorResultPoint;
import oimo.collision.narrowphase.detector.Detector;
import oimo.collision.narrowphase.detector.BoxBoxDetector;
import oimo.collision.narrowphase.detector.BoxBoxDetectorMacro;
import oimo.collision.narrowphase.detector.CachedDetectorData;
import oimo.collision.narrowphase.detector.CapsuleCapsuleDetector;
import oimo.collision.narrowphase.detector.GjkEpaDetector;
import oimo.collision.narrowphase.detector.SphereBoxDetector;
import oimo.collision.narrowphase.detector.SphereCapsuleDetector;
import oimo.collision.narrowphase.detector.SphereSphereDetector;
import oimo.collision.narrowphase.detector.gjkepa.EpaPolyhedron;
import oimo.collision.narrowphase.detector.gjkepa.EpaPolyhedronState;
import oimo.collision.narrowphase.detector.gjkepa.EpaTriangle;
import oimo.collision.narrowphase.detector.gjkepa.EpaVertex;
import oimo.collision.narrowphase.detector.gjkepa.GjkCache;
import oimo.common.Vec3;
import oimo.common.Transform;
import oimo.common.Setting;
import oimo.collision.narrowphase.detector.gjkepa.GjkEpa;
import oimo.collision.narrowphase.detector.gjkepa.GjkEpaLog;
import oimo.collision.narrowphase.detector.gjkepa.GjkEpaResultState;
import oimo.collision.narrowphase.detector.gjkepa.SimplexUtil;
import oimo.common.Mat3;
import oimo.common.Mat4;
import oimo.common.MathUtil;
import oimo.common.Pool;
import oimo.common.Quat;
import oimo.dynamics.Contact;
import oimo.dynamics.ContactLink;
import oimo.dynamics.ContactManager;
import oimo.dynamics.Island;
import oimo.dynamics.TimeStep;
import oimo.dynamics.World;
import oimo.dynamics.callback.AabbTestCallback;
import oimo.dynamics.callback.ContactCallback;
import oimo.dynamics.callback.RayCastCallback;
import oimo.dynamics.callback.RayCastClosest;
import oimo.dynamics.common.DebugDrawStyle;
import oimo.dynamics.common.Performance;
import oimo.dynamics.constraint.ConstraintSolver;
import oimo.dynamics.constraint.PositionCorrectionAlgorithm;
import oimo.dynamics.constraint.contact.ContactConstraint;
import oimo.dynamics.constraint.contact.ContactImpulse;
import oimo.dynamics.constraint.contact.Manifold;
import oimo.dynamics.constraint.contact.ManifoldPoint;
import oimo.dynamics.constraint.contact.ManifoldUpdater;
import oimo.dynamics.constraint.info.JacobianRow;
import oimo.dynamics.constraint.info.contact.ContactSolverInfo;
import oimo.dynamics.constraint.info.contact.ContactSolverInfoRow;
import oimo.dynamics.constraint.info.joint.JointSolverInfo;
import oimo.dynamics.constraint.info.joint.JointSolverInfoRow;
import oimo.dynamics.constraint.joint.BasisTracker;
import oimo.dynamics.constraint.joint.Joint;
import oimo.dynamics.constraint.joint.CylindricalJoint;
import oimo.dynamics.constraint.joint.JointConfig;
import oimo.dynamics.constraint.joint.CylindricalJointConfig;
import oimo.dynamics.constraint.joint.GenericJoint;
import oimo.dynamics.constraint.joint.GenericJointConfig;
import oimo.dynamics.constraint.joint.JointImpulse;
import oimo.dynamics.constraint.joint.JointLink;
import oimo.dynamics.constraint.joint.JointMacro;
import oimo.dynamics.constraint.joint.JointType;
import oimo.dynamics.constraint.joint.PrismaticJoint;
import oimo.dynamics.constraint.joint.PrismaticJointConfig;
import oimo.dynamics.constraint.joint.RagdollJoint;
import oimo.dynamics.constraint.joint.RagdollJointConfig;
import oimo.dynamics.constraint.joint.RevoluteJoint;
import oimo.dynamics.constraint.joint.RevoluteJointConfig;
import oimo.dynamics.constraint.joint.RotationalLimitMotor;
import oimo.dynamics.constraint.joint.SphericalJoint;
import oimo.dynamics.constraint.joint.SphericalJointConfig;
import oimo.dynamics.constraint.joint.SpringDamper;
import oimo.dynamics.constraint.joint.TranslationalLimitMotor;
import oimo.dynamics.constraint.joint.UniversalJoint;
import oimo.dynamics.constraint.joint.UniversalJointConfig;
import oimo.dynamics.constraint.solver.ConstraintSolverType;
import oimo.dynamics.constraint.solver.common.ContactSolverMassDataRow;
import oimo.dynamics.constraint.solver.common.JointSolverMassDataRow;
import oimo.dynamics.constraint.solver.direct.Boundary;
import oimo.dynamics.constraint.solver.direct.BoundaryBuildInfo;
import oimo.dynamics.constraint.solver.direct.BoundaryBuilder;
import oimo.dynamics.constraint.solver.direct.BoundarySelector;
import oimo.dynamics.constraint.solver.direct.DirectJointConstraintSolver;
import oimo.dynamics.constraint.solver.direct.MassMatrix;
import oimo.dynamics.constraint.solver.pgs.PgsContactConstraintSolver;
import oimo.dynamics.constraint.solver.pgs.PgsJointConstraintSolver;
import oimo.dynamics.rigidbody.MassData;
import oimo.dynamics.rigidbody.RigidBody;
import oimo.dynamics.rigidbody.RigidBodyConfig;
import oimo.dynamics.rigidbody.RigidBodyType;
import oimo.dynamics.rigidbody.Shape;
import oimo.dynamics.rigidbody.ShapeConfig;

/**
 * this class just imports all the classes in the library
 */
@:expose('Export')
class Export {
    public var _DebugDraw: DebugDraw;
    public var _BroadPhase: BroadPhase;
    public var _Geometry: Geometry;
    public var _ConvexGeometry: ConvexGeometry;
    public var _BroadPhaseProxyCallback: BroadPhaseProxyCallback;
    public var _BroadPhaseType: BroadPhaseType;
    public var _Proxy: Proxy;
    public var _ProxyPair: ProxyPair;
    public var _BruteForceBroadPhase: BruteForceBroadPhase;
    public var _BvhBroadPhase: BvhBroadPhase;
    public var _BvhInsertionStrategy: BvhInsertionStrategy;
    public var _BvhNode: BvhNode;
    public var _BvhProxy: BvhProxy;
    public var _BvhStrategy: BvhStrategy;
    public var _BvhTree: BvhTree;
    public var _Aabb: Aabb;
    public var _BoxGeometry: BoxGeometry;
    public var _CapsuleGeometry: CapsuleGeometry;
    public var _ConeGeometry: ConeGeometry;
    public var _ConvexHullGeometry: ConvexHullGeometry;
    public var _CylinderGeometry: CylinderGeometry;
    public var _GeometryType: GeometryType;
    public var _RayCastHit: RayCastHit;
    public var _SphereGeometry: SphereGeometry;
    public var _CollisionMatrix: CollisionMatrix;
    public var _DetectorResult: DetectorResult;
    public var _DetectorResultPoint: DetectorResultPoint;
    public var _Detector: Detector;
    public var _BoxBoxDetector: BoxBoxDetector;
    public var _BoxBoxDetectorMacro: BoxBoxDetectorMacro;
    public var _CachedDetectorData: CachedDetectorData;
    public var _CapsuleCapsuleDetector: CapsuleCapsuleDetector;
    public var _GjkEpaDetector: GjkEpaDetector;
    public var _SphereBoxDetector: SphereBoxDetector;
    public var _SphereCapsuleDetector: SphereCapsuleDetector;
    public var _SphereSphereDetector: SphereSphereDetector;
    public var _EpaPolyhedron: EpaPolyhedron;
    public var _EpaPolyhedronState: EpaPolyhedronState;
    public var _EpaTriangle: EpaTriangle;
    public var _EpaVertex: EpaVertex;
    public var _GjkCache: GjkCache;
    public var _Vec3: Vec3;
    public var _Transform: Transform;
    public var _Setting: Setting;
    public var _GjkEpa: GjkEpa;
    public var _GjkEpaLog: GjkEpaLog;
    public var _GjkEpaResultState: GjkEpaResultState;
    public var _SimplexUtil: SimplexUtil;
    public var _Mat3: Mat3;
    public var _Mat4: Mat4;
    public var _MathUtil: MathUtil;
    public var _Pool: Pool;
    public var _Quat: Quat;
    public var _Contact: Contact;
    public var _ContactLink: ContactLink;
    public var _ContactManager: ContactManager;
    public var _Island: Island;
    public var _TimeStep: TimeStep;
    public var _World: World;
    public var _AabbTestCallback: AabbTestCallback;
    public var _ContactCallback: ContactCallback;
    public var _RayCastCallback: RayCastCallback;
    public var _RayCastClosest: RayCastClosest;
    public var _DebugDrawStyle: DebugDrawStyle;
    public var _Performance: Performance;
    public var _ConstraintSolver: ConstraintSolver;
    public var _PositionCorrectionAlgorithm: PositionCorrectionAlgorithm;
    public var _ContactConstraint: ContactConstraint;
    public var _ContactImpulse: ContactImpulse;
    public var _Manifold: Manifold;
    public var _ManifoldPoint: ManifoldPoint;
    public var _ManifoldUpdater: ManifoldUpdater;
    public var _JacobianRow: JacobianRow;
    public var _ContactSolverInfo: ContactSolverInfo;
    public var _ContactSolverInfoRow: ContactSolverInfoRow;
    public var _JointSolverInfo: JointSolverInfo;
    public var _JointSolverInfoRow: JointSolverInfoRow;
    public var _BasisTracker: BasisTracker;
    public var _Joint: Joint;
    public var _CylindricalJoint: CylindricalJoint;
    public var _JointConfig: JointConfig;
    public var _CylindricalJointConfig: CylindricalJointConfig;
    public var _GenericJoint: GenericJoint;
    public var _GenericJointConfig: GenericJointConfig;
    public var _JointImpulse: JointImpulse;
    public var _JointLink: JointLink;
    public var _JointMacro: JointMacro;
    public var _JointType: JointType;
    public var _PrismaticJoint: PrismaticJoint;
    public var _PrismaticJointConfig: PrismaticJointConfig;
    public var _RagdollJoint: RagdollJoint;
    public var _RagdollJointConfig: RagdollJointConfig;
    public var _RevoluteJoint: RevoluteJoint;
    public var _RevoluteJointConfig: RevoluteJointConfig;
    public var _RotationalLimitMotor: RotationalLimitMotor;
    public var _SphericalJoint: SphericalJoint;
    public var _SphericalJointConfig: SphericalJointConfig;
    public var _SpringDamper: SpringDamper;
    public var _TranslationalLimitMotor: TranslationalLimitMotor;
    public var _UniversalJoint: UniversalJoint;
    public var _UniversalJointConfig: UniversalJointConfig;
    public var _ConstraintSolverType: ConstraintSolverType;
    public var _ContactSolverMassDataRow: ContactSolverMassDataRow;
    public var _JointSolverMassDataRow: JointSolverMassDataRow;
    public var _Boundary: Boundary;
    public var _BoundaryBuildInfo: BoundaryBuildInfo;
    public var _BoundaryBuilder: BoundaryBuilder;
    public var _BoundarySelector: BoundarySelector;
    public var _DirectJointConstraintSolver: DirectJointConstraintSolver;
    public var _MassMatrix: MassMatrix;
    public var _PgsContactConstraintSolver: PgsContactConstraintSolver;
    public var _PgsJointConstraintSolver: PgsJointConstraintSolver;
    public var _MassData: MassData;
    public var _RigidBody: RigidBody;
    public var _RigidBodyConfig: RigidBodyConfig;
    public var _RigidBodyType: RigidBodyType;
    public var _Shape: Shape;
    public var _ShapeConfig: ShapeConfig;
    
	static function main() {}
}