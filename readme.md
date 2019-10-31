<a name="createSimulation"></a>

## createSimulation(loop, element, wind, width, height, numberOfBottles, detectorAngle, detectorRange) ⇒ <code>\*</code>
Create a new simulation.

**Kind**: global function  

| Param | Type | Description |
| --- | --- | --- |
| loop | <code>function</code> | Function representing the Main control loop |
| element | <code>DOMElement</code> | DOM element to render the simulation canvas to. Default: document.body |
| wind | <code>Object</code> | a vector {x:number, y:number} |
| width | <code>number</code> | Width of the canvas |
| height | <code>number</code> | Height of the canvas |
| numberOfBottles | <code>number</code> | Number of bottles in the simulation |
| detectorAngle | <code>number</code> | Expected detection angle of the waste detector <code>-detectorAngle  <= x <= detectorAngle</code>. Default: 45° |
| detectorRange | <code>number</code> | Expected range of the detector. Default: 80 |

