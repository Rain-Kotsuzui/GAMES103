
using UnityEngine;
using System.Collections;
using System.Security.Cryptography.X509Certificates;
public class Rigid_Bunny : MonoBehaviour
{
	bool launched = false;
	Vector3 v = new Vector3(0, 0, 0);   // velocity
	Vector3 w = new Vector3(0, 0, 0);   // angular velocity
	float mass;                                 // mass
	Matrix4x4 I_ref;                            // reference inertia

	float linear_decay = 0.989f;                // for velocity decay
	float angular_decay = 0.989f;
	float restitution = 0.5f;                   // for collision


	// Use this for initialization
	void Start()
	{
		transform.position = new Vector3(0, 1.2f, 0);
		w = new Vector3(0, 2.0f, 0);

		Mesh mesh = GetComponent<MeshFilter>().mesh;
		Vector3[] vertices = mesh.vertices;

		float m = 1;
		mass = 0;
		for (int i = 0; i < vertices.Length; i++)
		{
			mass += m;
			float diag = m * vertices[i].sqrMagnitude;
			I_ref[0, 0] += diag;
			I_ref[1, 1] += diag;
			I_ref[2, 2] += diag;
			I_ref[0, 0] -= m * vertices[i][0] * vertices[i][0];
			I_ref[0, 1] -= m * vertices[i][0] * vertices[i][1];
			I_ref[0, 2] -= m * vertices[i][0] * vertices[i][2];
			I_ref[1, 0] -= m * vertices[i][1] * vertices[i][0];
			I_ref[1, 1] -= m * vertices[i][1] * vertices[i][1];
			I_ref[1, 2] -= m * vertices[i][1] * vertices[i][2];
			I_ref[2, 0] -= m * vertices[i][2] * vertices[i][0];
			I_ref[2, 1] -= m * vertices[i][2] * vertices[i][1];
			I_ref[2, 2] -= m * vertices[i][2] * vertices[i][2];
		}
		I_ref[3, 3] = 1;
	}

	Matrix4x4 Get_Cross_Matrix(Vector3 a)
	{
		//Get the cross product matrix of vector a
		Matrix4x4 A = Matrix4x4.zero;
		A[0, 0] = 0;
		A[0, 1] = -a[2];
		A[0, 2] = a[1];
		A[1, 0] = a[2];
		A[1, 1] = 0;
		A[1, 2] = -a[0];
		A[2, 0] = -a[1];
		A[2, 1] = a[0];
		A[2, 2] = 0;
		A[3, 3] = 1;
		return A;
	}

	// In this function, update v and w by the impulse due to the collision with
	//a plane <P, N>
	void Collision_Impulse(Vector3 P, Vector3 N)
	{
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		Vector3[] vertices = mesh.vertices;

		int colliding_vertices_count = 0;
		Vector3 sum_colliding_vertices_pos = Vector3.zero;

		for (int i = 0; i < vertices.Length; i++)
		{
			Vector3 r_i = transform.position + transform.rotation * vertices[i];
			float signed_distance = Vector3.Dot(r_i - P, N);
			if (signed_distance < 0)
			{
				Vector3 v_i = v + Vector3.Cross(w, r_i - transform.position);
				if (Vector3.Dot(v_i, N) < 0)
				{
					colliding_vertices_count++;
					sum_colliding_vertices_pos += r_i;
				}
			}
		}

		if (colliding_vertices_count == 0)
		{
			return;
		}

		Vector3 average_colliding_position = sum_colliding_vertices_pos / colliding_vertices_count;
		Vector3 r = average_colliding_position - transform.position;
		Vector3 v_avg_colliding_point = v + Vector3.Cross(w, r);

		float v_rel_n = Vector3.Dot(v_avg_colliding_point, N);
		if (v_rel_n >= 0)
		{
			return;
		}

		// I = R * I_ref * R^T
		Matrix4x4 R = Matrix4x4.Rotate(transform.rotation);
		Matrix4x4 I_inv = (R * I_ref * R.transpose).inverse;

		// K_term = ((I_inv * (r x N)) x r) dot N
		Vector3 r_cross_N = Vector3.Cross(r, N);
		Vector3 K_vec = I_inv.MultiplyVector(r_cross_N);
		float K_term = Vector3.Dot(Vector3.Cross(K_vec, r), N);

		// j = -(1+e)v_n / (1/m + K_term)
		float j = -(1 + restitution) * v_rel_n / (1 / mass + K_term);

		// For robustness, ensure we only apply a repulsive impulse (j > 0)
		j = Mathf.Max(0, j);

		Vector3 J = j * N;

		// v_new = v_old + J/m
		// w_new = w_old + I_inv * (r x J)
		v += J / mass;
		w += I_inv.MultiplyVector(Vector3.Cross(r, J));

	}

	// Update is called once per frame
	void Update()
	{
		//Game Control
		if (Input.GetKey("r"))
		{
			transform.position = new Vector3(0, 1.2f, 0);
			restitution = 0.5f;
			transform.rotation = Quaternion.identity; // 重置旋转
			v = Vector3.zero; // 重置线速度
			w = Vector3.zero; // 重置角速度
			launched = false;
		}
		if (Input.GetKeyDown("l"))
		{
			v = new Vector3(3f, 3f, 0) + new Vector3(Random.Range(-0.5f, 0.5f), Random.Range(-0.5f, 0.5f), Random.Range(-0.5f, 0.5f));
			launched = true;
		}
		if (Input.GetKeyDown("s"))
		{
			// 让 's' 键施加一个随机的角速度
			w = new Vector3(Random.Range(-5f, 5f), Random.Range(-5f, 5f), Random.Range(-5f, 5f));
		}

		// Part I: Update velocities
		if (launched)
		{
			v += new Vector3(0, -9.8f, 0) * Time.deltaTime;
			v *= linear_decay;
			w *= angular_decay;

			// Part II: Collision Impulse
			Collision_Impulse(new Vector3(0, 0.01f, 0), new Vector3(0, 1, 0));
			Collision_Impulse(new Vector3(2, 0, 0), new Vector3(-1, 0, 0));
		}
		// Part III: Update position & orientation

		//Update linear status
		Vector3 x = transform.position;
		//Update angular status
		Quaternion q = transform.rotation;
		x += v * Time.deltaTime;

		// 将角速度向量 w 转换为纯四元数
		Quaternion w_quat = new Quaternion(w.x, w.y, w.z, 0);

		// 计算四元数的变化量
		Quaternion delta_q = w_quat * q;

		// 使用一阶欧拉积分更新四元数
		q.x += Time.deltaTime * delta_q.x;
		q.y += Time.deltaTime * delta_q.y;
		q.z += Time.deltaTime * delta_q.z;
		q.w += Time.deltaTime * delta_q.w;

		// 归一化四元数，防止浮点数误差累积导致旋转不正常
		q.Normalize();

		// Part IV: Assign to the object
		transform.position = x;
		transform.rotation = q;
	}
	public void Anima(string str)
	{
		
	 }
}
