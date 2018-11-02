(define (domain turtlebot)

(:requirements :strips :typing :disjunctive-preconditions :fluents);:probabilistic-effects)

(:types
	waypoint 
	robot
)
(:functions (total-cost) (distance ?a ?b - waypoint))
(:predicates
	(robot_at ?v - robot ?wp - waypoint)
	(undocked ?v - robot)
	(docked ?v - robot)
	(localised ?v - robot)
	(dock_at ?wp - waypoint)

	;; Printing
	(printer_at ?wp - waypoint)
	(carrying_papers ?r - robot)
	(nocarrying_papers ?r - robot)
	(asked_load ?r - robot)
	(asked_unload ?r - robot)
	(papers_delivered ?r - robot ?w - waypoint)
	(DELIVERY_DESTINATION ?w - waypoint)
	(somebody_at ?w - waypoint)
)

;; Move to any waypoint, avoiding terrain
(:action goto_waypoint
	:parameters (?v - robot ?from ?to - waypoint)
	:precondition (and (robot_at ?v ?from) (localised ?v) (undocked ?v))
	:effect (and (increase (total-cost) (distance ?from ?to))
				(robot_at ?v ?to) (not (robot_at ?v ?from)) (not (asked_load ?v)) (not (asked_unload ?v)))
)

;; Localise
(:action localise
	:parameters (?v - robot)
	:precondition (undocked ?v)
	:effect (and (increase (total-cost) 10) (localised ?v))
)

;; Dock to charge
(:action dock
	:parameters (?v - robot ?wp - waypoint)
	:precondition (and
		(dock_at ?wp)
		(robot_at ?v ?wp)
		(undocked ?v))
	:effect (and (increase (total-cost) 5) (docked ?v) (not (undocked ?v)))
)

(:action undock
	:parameters (?v - robot ?wp - waypoint)
	:precondition (and
		(dock_at ?wp)
		(docked ?v))
	:effect (and (increase (total-cost) 0.5)
		(not (docked ?v))
		(undocked ?v))
)

(:action ask_load
	:parameters (?r - robot)
	:precondition (exists (?p - waypoint) (and (printer_at ?p) (nocarrying_papers ?r) (robot_at ?r ?p)))
	:effect (and (increase (total-cost) 0.5)
		(asked_load ?r))
)

(:action ask_unload
	:parameters (?r - robot)
	:precondition (exists (?w - waypoint) (and (DELIVERY_DESTINATION ?w) (carrying_papers ?r) (robot_at ?r ?w)))
	:effect (and (increase (total-cost) 0.5) (asked_unload ?r))
)

(:action wait_load
	:parameters (?r - robot)
	:precondition (and (asked_load ?r) (nocarrying_papers ?r) (exists (?p - waypoint) (and (somebody_at ?p) (printer_at ?p) (robot_at ?r ?p))))
	:effect (and (increase (total-cost) 0.25) 
		(carrying_papers ?r) (not (nocarrying_papers ?r)))
)

(:action wait_unload
	:parameters (?r - robot ?w - waypoint)
	:precondition (and (somebody_at ?w) (asked_unload ?r) (carrying_papers ?r) (DELIVERY_DESTINATION ?w) (robot_at ?r ?w))
	:effect (and (increase (total-cost) 0.25)
		(nocarrying_papers ?r) (not (carrying_papers ?r)) (papers_delivered ?r ?w)))
)
