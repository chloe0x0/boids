package scientificComputing;
import java.lang.Math;
import javax.swing.*;
import java.awt.*;
import java.util.Scanner;
import java.util.concurrent.ThreadLocalRandom;
import java.awt.Graphics2D;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Line2D;
import java.awt.Graphics2D;
import java.awt.geom.Rectangle2D;


class Vector2{
	private float x, y;
	public Vector2(float x, float y){
		this.x = x;
		this.y = y;
	}
	// dot product between Pascal vectors
	public float dotProduct(Vector2 b){
		return (x * b.getX()) + (y * b.getY());
	}
	// Boolean to evaluate wether two Pascal vectors are not equivalant
	public Boolean isNot(Vector2 b){
		return getX() != b.getX() && getY() != b.getY() ? true : false;  
	}
	// Boolean to evaluate wether two Pascal vectors are equivalant
	public Boolean is(Vector2 b){
		return getX() == b.getX() && getY() == b.getY() ? true : false;  
	}
	// Compute the sum of two Pascal Vectors
	public Vector2 vectorSum(Vector2 b){
		return new Vector2(getX() + b.getX(), getY() + b.getY());
	}
	// Compute the difference of two Pascal Vectors
	public Vector2 vectorDif(Vector2 b){
		return new Vector2(getX() - b.getX(), getY() - b.getY());
	}
	// Normalize the Pascal Vector and return its unit vector
	// normalized vector of a vector is the vector of magnitude 1 with the same orientation
	// given by the scalar quotient of the Pascal vector and its magnitude
	public Vector2 norm(){
		return scalarQuot(magnitude());
	}
	// Get scalar quotient between Pascal Vector and scalar
	public Vector2 scalarQuot(float scalar){
		return new Vector2(getX() / scalar, getY() / scalar);
	}
	// Get scalar product between Pascal Vector and scalar
	public Vector2 scalarProd(float scalar){
		return new Vector2(getX() * scalar, getY() * scalar);
	}
	// Get scalar difference bewteen Pascal Vector and scalar
	public Vector2 scalarDif(float scalar){
		return new Vector2(getX() - scalar, getY() - scalar);
	}
	// Get scalar sum between Pascal Vector and scalar
	public Vector2 scalarSum(float scalar){
		return new Vector2(getX() + scalar, getY() + scalar);
	}
	// Compute magnitude of a Pascal Vector
	public float magnitude(){
		return (float) Math.sqrt( (getX() * getX() ) + ( getY() * getY() ) );
	}
	// set magnitude of Pascal Vector
	public Vector2 setMag(float newMagnitude){
		float newX, newY;
		float magnitude = magnitude();
		newX = x * newMagnitude / magnitude;
		newY = y * newMagnitude / magnitude;
		return new Vector2(newX, newY);
	}	
	// clamp magnitude of Pascal Vector
	public Vector2 clamp(float max){
		float mag = magnitude();
		float f = Math.min(mag, max) / mag;
		return scalarProd(f);
	}

	// Getter methods to get X and Y components
	public float getX(){
		return this.x;
	}

	public float getY(){
		return this.y;
	}
	// Setter Methods to set X and Y components
	public void setX(float newX){
		this.x = newX;
	}

	public void setY(float newY){
		this.y = newY;
	}
	// display String representation
	public void asString(){
		System.out.println("<" + getX() + ", " + getY() + ">"); 
	}

	@Override
	public String toString(){
		return "<" + Float.toString(getX()) + ", " + Float.toString(getY()) + ">";
	}

	/* test main method. Use to debug methods
	
	public static void main(String[] args){
		Vector2 v = new Vector2(2f, 2f);
		Vector2 v2 = new Vector2(2f, 3f);
		System.out.println("Distance bewteen: ");
		v.asString();
		v2.asString();
		System.out.println(v.vectorDif(v2).magnitude());
	}
	*/
}


class boid{
	// will want to store two Pascal Vector2s, pos, and vel
	private Vector2 pos, vel;
	private int x, y;
	private float cohesionRadius, seperationRadius, alignmentRadius;
	private final float MAXSPEED = 10.0f;
	private final float MAXFORCE = 100.0f;
	public boid(int x, int y){
		this.pos = new Vector2(x, y);
		float velX, velY; 
		velX = (float)ThreadLocalRandom.current().nextInt(-5, 5);
		velY = (float)ThreadLocalRandom.current().nextInt(-5, 5);
		this.vel = new Vector2(velX, velY);
		this.seperationRadius = 5.0f;
		this.alignmentRadius = 100.0f;
		this.cohesionRadius = 50.0f;
	}
	/* first step in computing cohesion vector is to compute the "center of mass"
	 iterate over every boid that is not current boid, add vector to cohesion vector
	 divide by population - 1
	 difference between cohesionVector and current vector
	 scalar quotient of new vector and 100 (moves boid 1% to the center)
	 above will be a factor that can be messed with
	*/
	public Vector2 computeCohesion(boid[] population){
		Vector2 cohesionVector = new Vector2(0f, 0f);
		int n = 0;
		for (int i = 0; i < population.length; ++i){
			if (population[i].getPos().isNot(this.getPos())){
				float distance = ( population[i].getPos().vectorDif(this.getPos()).magnitude() ); 
				if (distance < cohesionRadius){
					cohesionVector = cohesionVector.vectorSum(population[i].getPos());
					++n;
				}
			}
		}
		if (n > 0){
			Vector2 desired, steerVector;
			desired = cohesionVector.vectorDif(this.getPos());
			desired = desired.norm();
			desired = desired.scalarProd(MAXSPEED);
			steerVector = desired.vectorSum(vel);
			return steerVector;
		}
		return new Vector2(0f, 0f);
	}

	public Vector2 computeSeperation(boid[] population){
		Vector2 seperationVector = new Vector2(0f, 0f);
		int n = 0;
		for (int i = 0; i < population.length; ++i){
			/*
			One line solution to condense logic block
			seperationVector =  distance < seperationRadius && population[i].isNot(this.pos) ? seperationVector.vectorDif( (population[i].pos.vectorDif(this.pos)) ) : seperationVector;
			*/
			if (population[i].getPos().isNot(this.getPos())){
				// distance between boids will be the magnitude of the difference of the position vectors
				float distance = ( population[i].getPos().vectorDif(this.getPos()).magnitude() );
				if (distance < seperationRadius){
					seperationVector = seperationVector.vectorDif( (population[i].getPos().vectorDif(this.getPos())) );
					seperationVector.norm();
					++n;
				}
			}
		}
		if (n > 0){
			return seperationVector.scalarQuot(n);
		}
		return new Vector2(0f, 0f);
	}

	public Vector2 computeAlignment(boid[] population){
		Vector2 alignmentVector = new Vector2(0f, 0f);
		int n = 0;
		for (int i = 0; i < population.length; ++i){
			if (population[i].getPos().isNot(this.getPos())){
				float distance = ( population[i].getPos().vectorDif(this.getPos()).magnitude() );
				if (distance < alignmentRadius){
					alignmentVector = alignmentVector.vectorSum(population[i].getVel());
					++n;
				}
			}
		}
		if (n > 0){
			alignmentVector = alignmentVector.scalarQuot(n);
			alignmentVector = alignmentVector.norm();
			alignmentVector = alignmentVector.scalarProd(MAXSPEED);
			alignmentVector = alignmentVector.vectorDif(vel);
			alignmentVector.clamp(MAXFORCE);
			return alignmentVector;
		}
		return new Vector2(0f, 0f);
	}

	public void move(boid[] population){
		Vector2 cohesion, alignment, seperation;
		cohesion = computeCohesion(population);
		seperation = computeSeperation(population);
		alignment = computeAlignment(population);
		alignment = alignment.scalarProd(1.5f);
		cohesion = cohesion.scalarProd(1.5f);
		pos = this.getPos();
		vel = this.getVel();
		//vel = vel.vectorSum(alignment).vectorSum(seperation).vectorSum(cohesion);
		vel = vel.vectorSum(seperation);
		vel = vel.vectorSum(cohesion);
		vel = vel.vectorSum(alignment);
		if (vel.magnitude() > MAXSPEED){
			vel = vel.scalarQuot(MAXSPEED);
		}
		pos = pos.vectorSum(this.getVel()); //.vectorSum(cohesion).vectorSum(seperation).vectorSum(alignment);
		setPos(pos);
	}

	public void setPos(Vector2 b){
		this.pos = b;
	}

	public Vector2 getPos(){
		return this.pos;
	}

	/*
	public Vector2 getAcc(){
		return this.acc;
	}
	*/
	public Vector2 getVel(){
		return this.vel;
	}

	public void inform(){
		Vector2 pos, vel;
		pos = getPos();
		vel = getVel();
		System.out.print("Boid at position: " + pos.getX() + " " + pos.getY() + " ");
		System.out.println("moving with velocity of " + vel.getX() + " " + vel.getY());
	}
}



class Panel extends JPanel{
	private boid[] population;
	private int count;
	private int width, height;
	// for now population will be fixed and inputed at cmd 
	// later use ArrayList for more dynamic populations
	public Panel(int width, int height, int count){
		this.setBackground(Color.BLACK);
		this.width = width;
		this.height = height;
		this.count = count;
		this.population = new boid[count];
		init_population(count);
	}

	public void init_population(int count){
		for (int i = 0; i < count; ++i){
			int x, y;
			x = ThreadLocalRandom.current().nextInt(10, getWidth());
			y = ThreadLocalRandom.current().nextInt(10, getHeight());
			boid b = new boid(x, y);
			population[i] = b;

		}
	}

	public void loop(){
		float width, height;
		width = (float) getWidth();
		height = (float) getHeight();
		for (int i = 0; i < getPopulation().length; ++i){
			getPopulation()[i].move(getPopulation());

			// Handle world wrapping
			if (getPopulation()[i].getPos().getX() > width){
			//	System.out.println(getPopulation()[i].getPos().getX() + " exceeds width: " + width);
				getPopulation()[i].getPos().setX(0f);
			}
			if (getPopulation()[i].getPos().getY() > height){
			//	System.out.println(getPopulation()[i].getPos().getY() + " exceeds height: " + height);
				getPopulation()[i].getPos().setY(0f);
			}

			if (getPopulation()[i].getPos().getX() < 0f){
			//	System.out.println(getPopulation()[i].getPos().getX() + " less than zero");
				getPopulation()[i].getPos().setX(width);
			}

			if (getPopulation()[i].getPos().getY() < 0f){
			//	System.out.println(getPopulation()[i].getPos().getY() + " less than zero" );
				getPopulation()[i].getPos().setY(height);
			}
		}
		repaint();
	}

	public int getWidth(){
		return width;
	}

	public int getHeight(){
		return height;
	}

	public boid[] getPopulation(){
		return population;
	}

	@Override
	public void paintComponent(Graphics g){
		final int WIDTH = 5; 
		Graphics2D g2 = (Graphics2D) g;
		super.paintComponent(g);
		g2.setColor(Color.WHITE);
		for (int i = 0; i < getPopulation().length; ++i){
			boid b = getPopulation()[i];
			double[] x, y;
			x = new double[] {
				b.getPos().getX() - WIDTH,
				b.getPos().getX(),
				b.getPos().getX() + WIDTH
			};
			y = new double[] {
				/* the vertex at the height of the triangle
				 computed as being the midpoint of the base line segment
				 summed to the velocity vector
				 in this case, the midpoint of the base line segment is <x, y>
				*/
				b.getPos().getX() + b.getVel().norm().getX(),
				b.getPos().getY() + b.getVel().norm().getY()
			};
			/* drawing base of the triangle
				then must draw lines from the base's vertices up to the 
				vertex at the height
			*/
			//g2.draw(new Line2D.Double(x[0], b.getPos().getY(), x[1], b.getPos().getY()));
			//g2.draw(new Line2D.Double(x[0], b.getPos().getY(), x[1], y[1]));
			//g2.draw(new Line2D.Double(x[2], b.getPos().getY(), x[1], y[1]));
			//g2.fill(new Ellipse2D.Double(b.getPos().getX(), b.getPos().getY(), 10, 10));
			
			 //g2.fill(new Rectangle2D.Double(
			// 	b.getPos().getX(),
			// 	b.getPos().getY() + b.getVel().norm().getY(),
			// 	(float)WIDTH,
			 //	WIDTH * 2f
			 //	));
			g2.draw(new Line2D.Double(b.getPos().getX(), b.getPos().getY(), b.getPos().getX(), b.getPos().getY()));
			//b.inform();
			/*
			Line2D.Double velocity = new Line2D.Double(
				b.getPos().getX(),
				b.getPos().getY(),
				b.getPos().getX() - b.getVel().getX(),
				b.getPos().getY() - b.getVel().getY()
				);
			g2.draw(velocity);
			*/
		//	g2.drawOval(b.getPos().getX(), b.getPos().getY(), 10, 10);
		}
	}
}

class Frame extends JFrame{
	private Panel p;
	public Frame(int width, int height, int count){
		p = new Panel(width, height, count);
		this.setSize(width, height);
		this.setTitle("Boids in Java");
		this.setResizable(false);
		this.getContentPane().add(p);
		this.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		this.setVisible(true);
	}
	public Panel getP(){
		return p;
	}
}

public class boids{
	public static void main(String[] args) throws InterruptedException{
		Scanner sc = new Scanner(System.in);
		System.out.println("Quantity of boids: ");
		int count = sc.nextInt();
		System.out.println("Width of screen: ");
		int width = sc.nextInt();
		System.out.println("Height of the screen");
		int height = sc.nextInt();

		Frame f = new Frame(width, height, count);

		while ( f.isVisible() ){
			Thread.sleep((long)25);
			f.getP().loop();
		}

	}
}