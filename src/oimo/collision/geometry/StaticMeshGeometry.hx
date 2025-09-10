package oimo.collision.geometry;
import haxe.ds.Vector;
import oimo.collision.broadphase.bvh.BvhTree;
import oimo.collision.broadphase.bvh.BvhProxy;
import oimo.common.MathUtil;
import oimo.common.Transform;
import oimo.common.Vec3;
import oimo.m.IVec3;
import oimo.m.M;

/**
 * Static mesh collision geometry.
 * Non-convex geometry that exposes triangle queries and raycast support.
 * Designed to work with specialized detectors (e.g. sphere vs static mesh).
 *
 * Performance: Uses BVH (Bounding Volume Hierarchy) for O(log n) triangle queries
 * instead of O(n) linear search. Suitable for meshes with thousands of triangles.
 */
@:build(oimo.m.B.bu())
class StaticMeshGeometry extends Geometry {
	public var _vertices:Vector<Vec3>;
	public var _indices:Vector<Int>;
	public var _normals:Vector<Vec3>;
	public var _numVertices:Int;
	public var _numTriangles:Int;
	var _triangleAABBs:Vector<Aabb>;
	var _triangleBVH:BvhTree;
	var _triangleProxies:Vector<BvhProxy>;
	var _nextProxyId:Int;

	public function new(vertices:Array<Vec3>, indices:Array<Int>, computeNormals:Bool = true) {
		super(GeometryType._STATIC_MESH);
		_numVertices = vertices.length;
		_numTriangles = Std.int(indices.length / 3);

		_vertices = new Vector<Vec3>(_numVertices);
		for (i in 0..._numVertices) _vertices[i] = new Vec3().copyFrom(vertices[i]);

		_indices = new Vector<Int>(indices.length);
		for (i in 0...indices.length) _indices[i] = indices[i];

		_normals = new Vector<Vec3>(_numTriangles);
		if (computeNormals) _computeTriangleNormals();
		else for (i in 0..._numTriangles) _normals[i] = new Vec3(0,1,0);

		_buildTriangleAABBs();
		_buildTriangleBVH();
		_updateMass();
	}

	public inline function getVertices():Vector<Vec3> return _vertices;
	public inline function getIndices():Vector<Int> return _indices;
	public inline function getNumTriangles():Int return _numTriangles;

	public function getTriangleVertices(triangleIndex:Int, v1:Vec3, v2:Vec3, v3:Vec3):Void {
		var idx = triangleIndex * 3;
		var i1 = _indices[idx];
		var i2 = _indices[idx + 1];
		var i3 = _indices[idx + 2];
		v1.copyFrom(_vertices[i1]);
		v2.copyFrom(_vertices[i2]);
		v3.copyFrom(_vertices[i3]);
	}

	public function getTriangleNormal(triangleIndex:Int, out:Vec3):Void {
		out.copyFrom(_normals[triangleIndex]);
	}

	public function getTriangleAABB(triangleIndex:Int):Aabb { return _triangleAABBs[triangleIndex]; }

	public function queryTriangles(aabb:Aabb):Array<Int> {
		var results:Array<Int> = [];

		// BVH-optimized query - O(log n)
		_queryBVHRecursive(_triangleBVH._root, aabb, results);

		return results;
	}

	function _queryBVHRecursive(node:oimo.collision.broadphase.bvh.BvhNode, queryAABB:Aabb, results:Array<Int>):Void {
		if (node == null) return;

		// Check if node's AABB overlaps with query AABB
		var nodeAABB = new Aabb();
		M.vec3_assign(nodeAABB._min, node._aabbMin);
		M.vec3_assign(nodeAABB._max, node._aabbMax);

		if (!nodeAABB.overlap(queryAABB)) return;

		if (node._height == 0) { // Leaf node
			// Add triangle index to results
			var triangleIndex:Int = cast node._proxy.userData;
			results.push(triangleIndex);
		} else { // Internal node
			// Recursively traverse children
			_queryBVHRecursive(node._children[0], queryAABB, results);
			_queryBVHRecursive(node._children[1], queryAABB, results);
		}
	}

	public function closestPointOnTriangle(point:Vec3, v1:Vec3, v2:Vec3, v3:Vec3, closestPoint:Vec3):{ u:Float, v:Float, w:Float } {
		// same algorithm used in many places; compute barycentric coords and closest point
		var edge1 = new Vec3().copyFrom(v2).subEq(v1);
		var edge2 = new Vec3().copyFrom(v3).subEq(v1);
		var v0 = new Vec3().copyFrom(v1).subEq(point);

		var a = edge1.dot(edge1);
		var b = edge1.dot(edge2);
		var c = edge2.dot(edge2);
		var d = edge1.dot(v0);
		var e = edge2.dot(v0);

		var det = a * c - b * b;
		var s = b * e - c * d;
		var t = b * d - a * e;

		if (s + t <= det) {
			if (s < 0) {
				if (t < 0) {
					if (d < 0) {
						t = 0;
						s = (-d >= a) ? 1 : -d / a;
					} else {
						s = 0;
						t = (e >= 0) ? 0 : ((-e >= c) ? 1 : -e / c);
					}
				} else {
					s = 0;
					t = (e >= 0) ? 0 : ((-e >= c) ? 1 : -e / c);
				}
			} else if (t < 0) {
				t = 0;
				s = (d >= 0) ? 0 : ((-d >= a) ? 1 : -d / a);
			} else {
				var invDet = 1 / det;
				s *= invDet;
				t *= invDet;
			}
		} else {
			if (s < 0) {
				var tmp0 = b + d;
				var tmp1 = c + e;
				if (tmp1 > tmp0) {
					var numer = tmp1 - tmp0;
					var denom = a - 2 * b + c;
					s = (numer >= denom) ? 1 : numer / denom;
					t = 1 - s;
				} else {
					s = 0;
					t = (tmp1 <= 0) ? 1 : ((e >= 0) ? 0 : -e / c);
				}
			} else if (t < 0) {
				var tmp0 = b + e;
				var tmp1 = a + d;
				if (tmp1 > tmp0) {
					var numer = tmp1 - tmp0;
					var denom = a - 2 * b + c;
					t = (numer >= denom) ? 1 : numer / denom;
					s = 1 - t;
				} else {
					t = 0;
					s = (tmp1 <= 0) ? 1 : ((d >= 0) ? 0 : -d / a);
				}
			} else {
				var numer = c + e - b - d;
				if (numer <= 0) s = 0; else {
					var denom = a - 2 * b + c;
					s = (numer >= denom) ? 1 : numer / denom;
				}
				t = 1 - s;
			}
		}

		var u = 1 - s - t;
		var v = s;
		var w = t;

		closestPoint.copyFrom(v1).scaleEq(u).addScaledEq(v2, v).addScaledEq(v3, w);
		return { u: u, v: v, w: w };
	}

	override public function _rayCastLocal(begin:IVec3, end:IVec3, hit:RayCastHit):Bool {
		var beginVec3 = new Vec3();
		var endVec3 = new Vec3();
		M.vec3_toVec3(beginVec3, begin);
		M.vec3_toVec3(endVec3, end);

		var rayDir = new Vec3().copyFrom(endVec3).subEq(beginVec3);
		var rayLength = rayDir.length();
		if (rayLength < 1e-6) return false;
		rayDir.scaleEq(1.0 / rayLength);

		var closestT = MathUtil.POSITIVE_INFINITY;
		var closestTriangle = -1;
		var closestU = 0.0;
		var closestV = 0.0;

		for (i in 0..._numTriangles) {
			var v1 = new Vec3(); var v2 = new Vec3(); var v3 = new Vec3();
			getTriangleVertices(i, v1, v2, v3);
			var rayHit = _rayTriangleIntersect(beginVec3, rayDir, v1, v2, v3);
			if (rayHit != null) {
				if (rayHit.t >= 0 && rayHit.t <= rayLength && rayHit.t < closestT) {
					closestT = rayHit.t;
					closestTriangle = i;
					closestU = rayHit.u;
					closestV = rayHit.v;
				}
			}
		}

		if (closestTriangle >= 0) {
			var hitPos = new Vec3().copyFrom(beginVec3).addScaledEq(rayDir, closestT);
			var hitNormal = new Vec3();
			getTriangleNormal(closestTriangle, hitNormal);

			var hitPosLocal:IVec3; var hitNormalLocal:IVec3;
			M.vec3_fromVec3(hitPosLocal, hitPos);
			M.vec3_fromVec3(hitNormalLocal, hitNormal);

			M.vec3_toVec3(hit.position, hitPosLocal);
			M.vec3_toVec3(hit.normal, hitNormalLocal);
			hit.fraction = closestT / rayLength;
			return true;
		}

		return false;
	}

	override public function _updateMass():Void {
		// approximate volume using AABB
		var minx = MathUtil.POSITIVE_INFINITY; var miny = MathUtil.POSITIVE_INFINITY; var minz = MathUtil.POSITIVE_INFINITY;
		var maxx = MathUtil.NEGATIVE_INFINITY; var maxy = MathUtil.NEGATIVE_INFINITY; var maxz = MathUtil.NEGATIVE_INFINITY;
		for (i in 0..._numVertices) {
			var v = _vertices[i];
			if (v.x < minx) minx = v.x; if (v.x > maxx) maxx = v.x;
			if (v.y < miny) miny = v.y; if (v.y > maxy) maxy = v.y;
			if (v.z < minz) minz = v.z; if (v.z > maxz) maxz = v.z;
		}
		var sizex = maxx - minx; var sizey = maxy - miny; var sizez = maxz - minz;
		_volume = sizex * sizey * sizez;
		M.mat3_diagonal(_inertiaCoeff,
			1 / 12 * (sizey * sizey + sizez * sizez),
			1 / 12 * (sizez * sizez + sizex * sizex),
			1 / 12 * (sizex * sizex + sizey * sizey)
		);
	}

	override public function _computeAabb(aabb:Aabb, tf:Transform):Void {
		var minx = MathUtil.POSITIVE_INFINITY; var miny = MathUtil.POSITIVE_INFINITY; var minz = MathUtil.POSITIVE_INFINITY;
		var maxx = MathUtil.NEGATIVE_INFINITY; var maxy = MathUtil.NEGATIVE_INFINITY; var maxz = MathUtil.NEGATIVE_INFINITY;
		for (i in 0..._numVertices) {
			var localV:IVec3; M.vec3_fromVec3(localV, _vertices[i]);
			var worldV:IVec3; M.vec3_mulMat3(worldV, localV, tf._rotation); M.vec3_add(worldV, worldV, tf._position);
			var x = M.vec3_get(worldV, 0); var y = M.vec3_get(worldV, 1); var z = M.vec3_get(worldV, 2);
			if (x < minx) minx = x; if (x > maxx) maxx = x;
			if (y < miny) miny = y; if (y > maxy) maxy = y;
			if (z < minz) minz = z; if (z > maxz) maxz = z;
		}
		M.vec3_set(aabb._min, minx, miny, minz);
		M.vec3_set(aabb._max, maxx, maxy, maxz);
	}

	function _computeTriangleNormals():Void {
		for (i in 0..._numTriangles) {
			var idx = i * 3;
			var i1 = _indices[idx]; var i2 = _indices[idx + 1]; var i3 = _indices[idx + 2];
			var v1 = _vertices[i1]; var v2 = _vertices[i2]; var v3 = _vertices[i3];
			var edge1 = new Vec3().copyFrom(v2).subEq(v1);
			var edge2 = new Vec3().copyFrom(v3).subEq(v1);
			var normal = new Vec3().copyFrom(edge1).crossEq(edge2).normalize();
			_normals[i] = normal;
		}
	}

	function _buildTriangleAABBs():Void {
		_triangleAABBs = new Vector<Aabb>(_numTriangles);
		for (i in 0..._numTriangles) {
			var v1 = new Vec3(); var v2 = new Vec3(); var v3 = new Vec3();
			getTriangleVertices(i, v1, v2, v3);
			var aabb = new Aabb();
			var minx = Math.min(v1.x, Math.min(v2.x, v3.x)); var miny = Math.min(v1.y, Math.min(v2.y, v3.y)); var minz = Math.min(v1.z, Math.min(v2.z, v3.z));
			var maxx = Math.max(v1.x, Math.max(v2.x, v3.x)); var maxy = Math.max(v1.y, Math.max(v2.y, v3.y)); var maxz = Math.max(v1.z, Math.max(v2.z, v3.z));
			M.vec3_set(aabb._min, minx, miny, minz); M.vec3_set(aabb._max, maxx, maxy, maxz);
			_triangleAABBs[i] = aabb;
		}
	}

	function _buildTriangleBVH():Void {
		_triangleBVH = new BvhTree();
		_triangleProxies = new Vector<BvhProxy>(_numTriangles);
		_nextProxyId = 0;

		// Create a BVH proxy for each triangle
		for (i in 0..._numTriangles) {
			var proxy = new BvhProxy(i, _nextProxyId++); // triangle index as userData
			proxy._setAabb(_triangleAABBs[i]);
			_triangleBVH._insertProxy(proxy);
			_triangleProxies[i] = proxy;
		}
	}

	function _rayTriangleIntersect(rayOrigin:Vec3, rayDir:Vec3, v1:Vec3, v2:Vec3, v3:Vec3):{ t:Float, u:Float, v:Float } {
		var edge1 = new Vec3().copyFrom(v2).subEq(v1);
		var edge2 = new Vec3().copyFrom(v3).subEq(v1);
		var h = new Vec3().copyFrom(rayDir).crossEq(edge2);
		var a = edge1.dot(h);
		if (a > -1e-6 && a < 1e-6) return null;
		var f = 1.0 / a;
		var s = new Vec3().copyFrom(rayOrigin).subEq(v1);
		var u = f * s.dot(h);
		if (u < 0.0 || u > 1.0) return null;
		var q = new Vec3().copyFrom(s).crossEq(edge1);
		var v = f * rayDir.dot(q);
		if (v < 0.0 || u + v > 1.0) return null;
		var t = f * edge2.dot(q);
		if (t > 1e-6) return { t: t, u: u, v: v };
		return null;
	}
}
