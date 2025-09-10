package oimo.collision.narrowphase.detector;
import oimo.collision.geometry.*;
import oimo.collision.narrowphase.*;
import oimo.collision.narrowphase.detector.gjkepa.*;
import oimo.common.Transform;
import oimo.common.Vec3;
import oimo.m.IVec3;
import oimo.m.M;

/**
 * ConvexGeometry vs StaticMeshGeometry collision detector.
 */
@:build(oimo.m.B.bu())
class ConvexStaticMeshDetector extends Detector {

	public function new(swapped:Bool = false) {
		super(swapped);
	}

	override function detectImpl(result:DetectorResult, geom1:Geometry, geom2:Geometry, tf1:Transform, tf2:Transform, cachedData:CachedDetectorData):Void {
		var convex:ConvexGeometry = cast geom1;
		var staticMesh:StaticMeshGeometry = cast geom2;

		result.incremental = false;

		// Query potentially colliding triangles using world-space AABB
		var aabb:Aabb = new Aabb();
		convex._computeAabb(aabb, tf1);
		var triangleIndices:Array<Int> = staticMesh.queryTriangles(aabb);

		// Early exit if no triangles
		if (triangleIndices.length == 0) {
			return;
		}

		var gjkEpa:GjkEpa = GjkEpa.getInstance();
		var bestPenetrationDepth:Float = -Math.POSITIVE_INFINITY;
		var bestContactFound:Bool = false;
		var bestNormalWorld:Vec3 = new Vec3();
		var bestContactPointConvex:Vec3 = new Vec3();
		var bestContactPointMesh:Vec3 = new Vec3();

		// Test each potentially colliding triangle using GJK/EPA
		for (triangleIndex in triangleIndices) {
			var v1 = new Vec3();
			var v2 = new Vec3();
			var v3 = new Vec3();
			staticMesh.getTriangleVertices(triangleIndex, v1, v2, v3);

			// Create triangle geometry using ConvexHullGeometry
			var triangleVertices:Array<Vec3> = [v1, v2, v3];
			var triangleGeom = new ConvexHullGeometry(triangleVertices);
			triangleGeom._gjkMargin = 0.0; // Triangle has no margin

			// Use GJK/EPA to compute collision between convex and triangle
			// Triangle is already in mesh local space, so use tf2 directly
			var status:Int = gjkEpa.computeClosestPoints(convex, triangleGeom, tf1, tf2, null);

			if (status != GjkEpaResultState.SUCCEEDED) {
				continue; // Skip this triangle if GJK/EPA failed
			}

			var margin1:Float = convex._gjkMargin;
			var margin2:Float = triangleGeom._gjkMargin;

			if (gjkEpa.distance > margin1 + margin2) {
				continue; // No collision with this triangle
			}

			// Calculate penetration depth (similar to GjkEpaDetector)
			var penetrationDepth:Float = margin1 + margin2 - gjkEpa.distance;

			// Keep track of the deepest penetration
			if (penetrationDepth > bestPenetrationDepth) {
				bestPenetrationDepth = penetrationDepth;
				bestContactFound = true;

				// Calculate collision normal (from triangle to convex)
				bestNormalWorld.copyFrom(gjkEpa.closestPoint1).subEq(gjkEpa.closestPoint2);
				if (gjkEpa.distance < 0) {
					bestNormalWorld.negateEq();
				}
				bestNormalWorld.normalize();

				// Calculate contact points (similar to GjkEpaDetector)
				bestContactPointConvex.copyFrom(gjkEpa.closestPoint1).addScaledEq(bestNormalWorld, -margin1);
				bestContactPointMesh.copyFrom(gjkEpa.closestPoint2).addScaledEq(bestNormalWorld, margin2);
			}
		}

		// If we found a collision, create the contact
		if (bestContactFound) {
			var normalInWorld:IVec3;
			M.vec3_fromVec3(normalInWorld, bestNormalWorld);

			// Set collision normal (this handles the swapped case automatically)
			setNormal(result, M.vec3_get(normalInWorld, 0), M.vec3_get(normalInWorld, 1), M.vec3_get(normalInWorld, 2));

			var contactPointConvex:IVec3;
			var contactPointMesh:IVec3;
			M.vec3_fromVec3(contactPointConvex, bestContactPointConvex);
			M.vec3_fromVec3(contactPointMesh, bestContactPointMesh);

			// Add contact point (this handles the swapped case automatically)
			addPoint(result,
				M.vec3_get(contactPointConvex, 0), M.vec3_get(contactPointConvex, 1), M.vec3_get(contactPointConvex, 2),
				M.vec3_get(contactPointMesh, 0), M.vec3_get(contactPointMesh, 1), M.vec3_get(contactPointMesh, 2),
				bestPenetrationDepth, 0);
		}
	}
}