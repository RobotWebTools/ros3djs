const express = require('express')
const app = express()

app.use(express.static('.'))

const examples = {
  markers: './markers.html'
}

app.get('/', (req, res) => res.send([
  `<p>These examples demonstrate how to use ROS3D's es6 module within an html script tag:</p>`,
  `<ul>`,
  ...Object.entries(examples).map(([example, link]) => {
    return `<li><a href="${link}">${example}</a></li>`
  }),
  `</ul>`,
].join('\n')))

app.listen(3000, () => console.log('Example app listening on port 3000!'))
