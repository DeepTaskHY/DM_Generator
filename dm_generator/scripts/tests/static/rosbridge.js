'use strict'

$(document).ready(() => {
    var submitMessage = (message = {}, mine = true) => {
        var $wrap = $('#dialog-wrap')

        var $example = $wrap
            .find('.dialog-example')

        var $dialog = $example.clone()
            .removeClass('dialog-example visually-hidden')
            .appendTo($wrap)

        var $dialogOutputWrap = $dialog
            .children('.dialog-output')

        var $dialogOutput = $dialogOutputWrap
            .children('button')

        var $dialogDetailWrap = $dialog
            .children('.dialog-detail')

        var $dialogDetail = $dialogDetailWrap
            .children('.card')

        // Set class style
        if (mine) {
            $dialogOutputWrap
                .addClass('justify-content-start')

            $dialogOutput
                .addClass('btn-primary')
        }
        else {
            $dialogOutputWrap
                .addClass('justify-content-end')

            $dialogOutput
                .addClass('btn-dark')
        }

        // Set dialog detail ID
        var id = $wrap.children('.dialog').length - 1
        var detailName = $dialogOutput.attr('aria-controls')
        var detailId = `${detailName}-${id}`

        $dialogOutput
            .attr('data-bs-target', `#${detailId}`)
            .attr('aria-controls', detailId)

        $dialogDetailWrap
            .attr('id', detailId)

        // Set message
        var header = message['header']
        var contentName = header['content']
        var content = message[contentName]

        if (mine)
            $dialogOutput
                .text(content['human_speech'])
        else
            $dialogOutput
                .text(content['dialog'])

        var $messageWrap = $('<pre/>')
            .text(JSON.stringify(message, null, 2))

        $dialogDetail.append($messageWrap)

        // Set scroll position
        $wrap.scrollTop($wrap.prop('scrollHeight'))

        return $dialog
    }

    var sio = io('/rosbridge')

    // Publish message
    $('#input-form').on('submit', (e) => {
        e.preventDefault()

        var data = $(e.currentTarget).serializeObject()
        var header = data['header']
        var contentName = header['content']
        delete data['header']

        var message = {
            'header': header,
            [contentName]: data
        }

        // Publish
        sio.emit('publish', {'data': JSON.stringify(message)})
        submitMessage(message, true)
    })

    // Subscribe message
    sio.on('subscribe', (data) => {
        var message = JSON.parse(data['data'])

        // Print message
        submitMessage(message, false)

        // Increase message ID
        $('#id').val((i, val) => { return ++val })
    })
})
