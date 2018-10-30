(define (problem task)
(:domain turtlebot)
(:objects
    entrance dock-station meeting-room dan-office phdarea fridge coffee printer-ent printer-corridor printer-phdarea - waypoint
    kenny - robot
)
(:init
    (robot_at kenny dock-station)
    (docked kenny)
    (dock_at dock-station)
    (nocarrying_papers kenny)
    (printer_at printer-ent)
    (printer_at printer-corridor)
    (printer_at printer-phdarea)

    (DELIVERY_DESTINATION dan-office)
    (= (distance dock-station printer-corridor) 1)
    (= (distance dock-station printer-ent) 1)
    (= (distance printer-corridor dan-office) 1)
    (= (distance printer-ent dan-office) 2)
    (= (distance dan-office dock-station) 1)
    (= (total-cost) 0)
)
(:goal (and
    (papers_delivered kenny dan-office)
    (docked kenny)
    )
)
(:metric minimize (total-cost))
)
