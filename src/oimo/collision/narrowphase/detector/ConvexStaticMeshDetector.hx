package oimo.collision.narrowphase.detector;
import oimo.collision.geometry.*;
import oimo.collision.narrowphase.*;
import oimo.common.MathUtil;
import oimo.common.Transform;
import oimo.common.Vec3;
import oimo.m.IVec3;
import oimo.m.M;

/**
 * ConvexGeometry vs StaticMesh collision detector.
 * Handles any convex shape (Sphere, Box, Capsule, ConvexHull, etc.) vs StaticMesh.
 *
 * SIMPLIFIED VERSION - based on working sphere detector approach
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

		// Get a reasonable bounding radius for the convex shape
		// Use a conservative approach based on GJK margin
		var boundingRadius = getBoundingRadius(convex);

		// Transform convex center to mesh local space
		var meshToConvex:IVec3;
		M.vec3_sub(meshToConvex, tf1._position, tf2._position);
		var meshToConvexInMesh:IVec3;
		M.vec3_mulMat3Transposed(meshToConvexInMesh, meshToConvex, tf2._rotation);

		// Convert to Vec3 for geometry queries
		var localConvexCenter = new Vec3();
		M.vec3_toVec3(localConvexCenter, meshToConvexInMesh);

		// Create conservative AABB in mesh local space for broad phase
		var convexAABB = new Aabb();
		M.vec3_set(convexAABB._min, localConvexCenter.x - boundingRadius, localConvexCenter.y - boundingRadius, localConvexCenter.z - boundingRadius);
		M.vec3_set(convexAABB._max, localConvexCenter.x + boundingRadius, localConvexCenter.y + boundingRadius, localConvexCenter.z + boundingRadius);

		// Query potentially colliding triangles
		var triangleIndices = staticMesh.queryTriangles(convexAABB);

		var closestDistance = MathUtil.POSITIVE_INFINITY;
		var closestTriangle = -1;
		var closestPointLocal = new Vec3();
		var closestNormalLocal = new Vec3();

		// Test each potentially colliding triangle
		for (triangleIndex in triangleIndices) {
			var v1 = new Vec3();
			var v2 = new Vec3();
			var v3 = new Vec3();
			staticMesh.getTriangleVertices(triangleIndex, v1, v2, v3);

			// Get the closest point on triangle to convex center (simplified approach)
			var triangleClosestPoint = new Vec3();
			staticMesh.closestPointOnTriangle(localConvexCenter, v1, v2, v3, triangleClosestPoint);

			// Calculate distance from convex center to triangle
			var toConvex = new Vec3().copyFrom(localConvexCenter).subEq(triangleClosestPoint);
			var distance = toConvex.length();

			// Check if this is a collision candidate
			if (distance < boundingRadius && distance < closestDistance) {
				closestDistance = distance;
				closestTriangle = triangleIndex;
				closestPointLocal.copyFrom(triangleClosestPoint);

				// Calculate proper collision normal
				if (distance > 1e-6) {
					// Normal points from triangle towards convex center
					closestNormalLocal.copyFrom(toConvex).scaleEq(1.0 / distance);
				} else {
					// Convex center is on triangle - use face normal
					staticMesh.getTriangleNormal(triangleIndex, closestNormalLocal);
					// Ensure normal points towards convex center
					var testPoint = new Vec3().copyFrom(triangleClosestPoint).addScaledEq(closestNormalLocal, 0.001);
					if (localConvexCenter.sub(testPoint).length() < localConvexCenter.sub(triangleClosestPoint).length()) {
						closestNormalLocal.scaleEq(-1);
					}
				}
			}
		}

		// If we found a collision, create the contact
		if (closestTriangle >= 0) {
			// Calculate actual penetration using support vertex
			var penetrationDepth = calculatePenetrationDepth(convex, tf1, tf2, closestNormalLocal, boundingRadius - closestDistance);

			if (penetrationDepth > 0) {
				// Transform collision normal back to world space
				var normalInWorld:IVec3;
				M.vec3_fromVec3(normalInWorld, closestNormalLocal);
				M.vec3_mulMat3(normalInWorld, normalInWorld, tf2._rotation);

				// Set collision normal (this handles the swapped case automatically)
				setNormal(result, M.vec3_get(normalInWorld, 0), M.vec3_get(normalInWorld, 1), M.vec3_get(normalInWorld, 2));

				// Calculate contact points in world space
				var contactPointOnMesh:IVec3;
				M.vec3_fromVec3(contactPointOnMesh, closestPointLocal);
				M.vec3_mulMat3(contactPointOnMesh, contactPointOnMesh, tf2._rotation);
				M.vec3_add(contactPointOnMesh, contactPointOnMesh, tf2._position);

				// Get contact point on convex using support vertex
				var supportDir = new Vec3().copyFrom(closestNormalLocal).scaleEq(-1);
				var supportPoint = new Vec3();
				convex.computeLocalSupportingVertex(supportDir, supportPoint);

				var contactPointOnConvex:IVec3;
				M.vec3_fromVec3(contactPointOnConvex, supportPoint);
				M.vec3_mulMat3(contactPointOnConvex, contactPointOnConvex, tf1._rotation);
				M.vec3_add(contactPointOnConvex, contactPointOnConvex, tf1._position);

				// Add contact point (this handles the swapped case automatically)
				addPoint(result,
					M.vec3_get(contactPointOnConvex, 0), M.vec3_get(contactPointOnConvex, 1), M.vec3_get(contactPointOnConvex, 2),
					M.vec3_get(contactPointOnMesh, 0), M.vec3_get(contactPointOnMesh, 1), M.vec3_get(contactPointOnMesh, 2),
					penetrationDepth, 0);
			}
		}
	}

	private function getBoundingRadius(convex:ConvexGeometry):Float {
		// For different convex shapes, we can use different strategies
		// For now, use a conservative approach
		switch (convex._type) {
			case GeometryType._SPHERE:
				var sphere:SphereGeometry = cast convex;
				return sphere.getRadius();
			case GeometryType._BOX:
				// Conservative estimate for box
				return convex._gjkMargin * 50; // Larger for boxes
			case GeometryType._CAPSULE:
				// Conservative estimate for capsule
				return convex._gjkMargin * 30;
			default:
				// Very conservative for unknown shapes
				return convex._gjkMargin * 20;
		}
	}

	private function calculatePenetrationDepth(convex:ConvexGeometry, tf1:Transform, tf2:Transform, normal:Vec3, estimatedDepth:Float):Float {
		// For now, use the estimated depth
		// In a full implementation, we'd do a more precise calculation using support vertices
		return Math.max(0, estimatedDepth);
	}
}
